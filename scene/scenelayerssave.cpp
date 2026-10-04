/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/scenelayers.h"
#include "world/Track.h"
#include "world/Road.h"
#include "world/RoadPoint.h"

#include "simulation/simulation.h"
#include "model/AnimModel.h"
#include "world/MemCell.h"
#include "world/Traction.h"
#include "world/EvLaunch.h"
#include "world/Event.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <map>
#include <optional>
#include <set>

// scenery save for sceneries opened for editing.
// the scenery files are patched rather than generated: text of a file is carried over as it is, except for the
// definitions of nodes changed in the editor and the include directives affected by layer changes. this keeps
// everything the editor doesn't understand or didn't touch, comments and formatting included.

namespace scene
{

namespace
{

struct text_token
{
	std::size_t begin;
	std::size_t end;
	std::string text; // lower case
};

struct text_change
{
	std::size_t begin;
	std::size_t end;
	std::string text;
};

bool is_separator(char const Char)
{
	return Char == ' ' || Char == '\t' || Char == '\n' || Char == '\r' || Char == ';';
}

// splits scenery text into tokens the same way the scenery parser does: comments are skipped, quoted text is kept together
std::vector<text_token> tokenize(std::string const &Text)
{
	std::vector<text_token> tokens;
	auto const size{Text.size()};
	std::size_t idx{0};
	auto const starts = [&](char const *Mark) { return Text.compare(idx, 2, Mark) == 0; };
	while (idx < size)
	{
		if (is_separator(Text[idx]))
		{
			++idx;
			continue;
		}
		if (starts("//"))
		{
			idx = Text.find('\n', idx);
			idx = (idx == std::string::npos ? size : idx + 1);
			continue;
		}
		if (starts("/*"))
		{
			idx = Text.find("*/", idx + 2);
			idx = (idx == std::string::npos ? size : idx + 2);
			continue;
		}
		auto const begin{idx};
		while (idx < size && false == is_separator(Text[idx]) && false == starts("//") && false == starts("/*"))
		{
			if (Text[idx] == '\"')
			{
				// quoted part runs up to the closing quote, whatever is inside
				++idx;
				while (idx < size && Text[idx] != '\"')
				{
					idx += (Text[idx] == '\\' ? 2 : 1);
				}
			}
			++idx;
		}
		idx = std::min(idx, size);
		tokens.push_back({begin, idx, ToLower(Text.substr(begin, idx - begin))});
	}
	return tokens;
}

std::string number(double const Value)
{
	std::ostringstream converter;
	converter.imbue(std::locale::classic());
	converter << std::fixed << std::setprecision(3) << Value;
	auto text{converter.str()};
	return text == "-0.000" ? "0.000" : text;
}

std::string numbers(glm::dvec3 const &Values)
{
	return number(Values.x) + ' ' + number(Values.y) + ' ' + number(Values.z);
}

void apply(std::string &Text, std::vector<text_change> Changes)
{
	// from the end, so the locations of pending changes stay valid
	std::sort(std::begin(Changes), std::end(Changes), [](text_change const &Left, text_change const &Right) { return Left.begin > Right.begin; });
	for (auto const &change : Changes)
	{
		Text.replace(change.begin, change.end - change.begin, change.text);
	}
}

// placement to write into definition of a model instance. empty parts are left as they are
struct model_placement
{
	std::optional<glm::dvec3> location; // in the coordinates used by the definition
	std::optional<glm::vec3> angles; // complete rotation of the instance
	glm::vec3 rotation{0.f}; // rotation the loader adds to the angle specified in the definition
	std::optional<glm::vec3> scale; // scale factor to put in the definition
};

bool about_equal(float const Left, float const Right)
{
	return std::abs(Left - Right) < 1e-3f;
}

// definition layout: node <max> <min> <name> model <x> <y> <z> <angle> <model file> <texture> [options] endmodel
bool patch_model(std::string &Text, model_placement const &Placement, std::string &Error)
{
	auto const tokens{tokenize(Text)};
	if (tokens.size() < 12 || tokens[0].text != "node" || tokens[4].text != "model" || tokens.back().text != "endmodel")
	{
		Error = "unexpected layout of model definition";
		return false;
	}
	std::vector<text_change> changes;
	auto const replace = [&](std::size_t const Index, std::string const &Value) { changes.push_back({tokens[Index].begin, tokens[Index].end, Value}); };
	// optional blocks of three values, located past the texture
	auto const last{tokens.size() - 1};
	auto const find_block = [&](char const *Keyword) {
		for (std::size_t idx = 11; idx + 3 < last; ++idx)
		{
			if (tokens[idx].text == Keyword)
			{
				return idx;
			}
		}
		return std::size_t{0};
	};
	std::string additions; // blocks to insert ahead of the closing keyword

	if (Placement.location)
	{
		replace(5, number(Placement.location->x));
		replace(6, number(Placement.location->y));
		replace(7, number(Placement.location->z));
	}
	if (Placement.angles)
	{
		auto const &angles{*Placement.angles};
		auto const block{find_block("angles")};
		if (block != 0)
		{
			// the block specifies complete rotation of the instance, regardless of anything else
			replace(block + 1, number(angles.x));
			replace(block + 2, number(angles.y));
			replace(block + 3, number(angles.z));
		}
		else if (about_equal(angles.x, Placement.rotation.x) && about_equal(angles.z, Placement.rotation.z))
		{
			// rotation around the vertical axis alone fits in the basic angle parameter
			replace(8, number(angles.y - Placement.rotation.y));
		}
		else
		{
			additions += "angles " + numbers(angles) + ' ';
		}
	}
	if (Placement.scale)
	{
		auto const &scale{*Placement.scale};
		auto const unit{about_equal(scale.x, 1.f) && about_equal(scale.y, 1.f) && about_equal(scale.z, 1.f)};
		auto const block{find_block("scale")};
		if (block != 0)
		{
			if (unit)
			{
				// drop the block along with the whitespace separating it from the next token
				changes.push_back({tokens[block].begin, tokens[block + 4].begin, ""});
			}
			else
			{
				replace(block + 1, number(scale.x));
				replace(block + 2, number(scale.y));
				replace(block + 3, number(scale.z));
			}
		}
		else if (false == unit)
		{
			additions += "scale " + numbers(scale) + ' ';
		}
	}
	if (false == additions.empty())
	{
		changes.push_back({tokens[last].begin, tokens[last].begin, additions});
	}
	apply(Text, changes);
	return true;
}

// definition layout: node <max> <min> <name> memcell <x> <y> <z> <text> <value 1> <value 2> <track> endmemcell
bool patch_memcell(std::string &Text, glm::dvec3 const &Location, std::string &Error)
{
	auto const tokens{tokenize(Text)};
	if (tokens.size() < 13 || tokens[0].text != "node" || tokens[4].text != "memcell" || tokens.back().text != "endmemcell")
	{
		Error = "unexpected layout of memory cell definition";
		return false;
	}
	apply(Text, {{tokens[5].begin, tokens[5].end, number(Location.x)}, {tokens[6].begin, tokens[6].end, number(Location.y)}, {tokens[7].begin, tokens[7].end, number(Location.z)}});
	return true;
}

// definition layout: node <max> <min> <name> traction <supply> <voltage> <current> <resistivity> <material> <thickness> <damage>
//   <point 1> <point 2> <point 3> <point 4> <minimal height> <segment length> <wires> <offset> <vis> [parallel <name>] endtraction
bool patch_traction(std::string &Text, TTraction const &Traction, glm::dvec3 const &Offset, std::string &Error)
{
	auto const tokens{tokenize(Text)};
	if (tokens.size() < 30 || tokens[0].text != "node" || tokens[4].text != "traction" || tokens.back().text != "endtraction")
	{
		Error = "unexpected layout of traction definition";
		return false;
	}
	std::vector<text_change> changes;
	std::size_t index{12};
	for (auto const &point : {Traction.pPoint1, Traction.pPoint2, Traction.pPoint3, Traction.pPoint4})
	{
		auto const local{point - Offset};
		for (auto const value : {local.x, local.y, local.z})
		{
			changes.push_back({tokens[index].begin, tokens[index].end, number(value)});
			++index;
		}
	}
	auto const &p1{Traction.pPoint1};
	auto const &p2{Traction.pPoint2};
	auto const &p3{Traction.pPoint3};
	auto const &p4{Traction.pPoint4};
	changes.push_back({tokens[24].begin, tokens[24].end, number((p3.y - p1.y + p4.y - p2.y) * 0.5 - Traction.fHeightDifference)});
	if (Traction.iNumSections > 0)
	{
		// the load truncates length / segment length, the margin keeps the rounding of the number from losing a section
		changes.push_back({tokens[25].begin, tokens[25].end, number(glm::length(p1 - p2) / (Traction.iNumSections + 0.001))});
	}
	apply(Text, changes);
	return true;
}

// definition layout: node <max> <min> <name> eventlauncher <x> <y> <z> <radius> ... end
bool patch_launcher(std::string &Text, glm::dvec3 const &Location, std::string &Error)
{
	auto const tokens{tokenize(Text)};
	if (tokens.size() < 9 || tokens[0].text != "node" || tokens[4].text != "eventlauncher")
	{
		Error = "unexpected layout of event launcher definition";
		return false;
	}
	apply(Text, {{tokens[5].begin, tokens[5].end, number(Location.x)}, {tokens[6].begin, tokens[6].end, number(Location.y)}, {tokens[7].begin, tokens[7].end, number(Location.z)}});
	return true;
}

std::string detect_eol(std::string const &Text, std::string const &Fallback)
{
	auto const linebreak{Text.find('\n')};
	if (linebreak == std::string::npos)
	{
		return Fallback;
	}
	return (linebreak > 0 && Text[linebreak - 1] == '\r') ? "\r\n" : "\n";
}

void ensure_newline(std::string &Text, std::string const &Eol)
{
	if (false == Text.empty() && Text.back() != '\n')
	{
		Text += Eol;
	}
}

bool read_file(std::string const &Path, std::string &Content)
{
	std::ifstream input{Path, std::ios_base::binary};
	if (false == input.is_open())
	{
		return false;
	}
	Content.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
	return true;
}

} // namespace

// changes to make in the text of a single scenery file
struct node_layers::file_patch
{
	struct edit
	{
		std::streamoff begin{0}; // replaced range of the original text. empty range inserts, negative location stands for the end of the text
		std::streamoff end{0};
		std::string text; // replacement
		std::size_t lead{0}; // size of the part of replacement preceding the definition it carries
		std::size_t length{0}; // size of that definition
		basic_node const *node{nullptr}; // node defined by the replacement
		layer_handle site{null_handle}; // layer included by the directive in the replacement
		instance_handle instance{0}; // include of a template made by the directive in the replacement
		layer_handle inlined{null_handle}; // layer whose content replaces the range
	};
	std::vector<edit> edits;
};

// text of a scenery file after the changes, with the new locations of what it defines
struct node_layers::composition
{
	struct node_place
	{
		basic_node const *node;
		source_span span;
		bool rewritten;
	};
	struct site_place
	{
		layer_handle layer;
		source_span span;
	};
	struct instance_place
	{
		instance_handle instance;
		source_span span;
	};
	std::string text;
	std::vector<node_place> nodes;
	std::vector<site_place> sites;
	std::vector<instance_place> instances;
	source_span init; // location of what performs the scenario initialization, see basic_layer::init

	// adds another piece of text at the end
	void append(composition const &Other)
	{
		insert(static_cast<std::streamoff>(text.size()), Other);
	}
	// adds another piece of text at specified location
	void insert(std::streamoff const Location, composition const &Other)
	{
		auto const size{static_cast<std::streamoff>(Other.text.size())};
		auto const move = [=](source_span &Span) {
			if (Span.begin >= Location)
			{
				Span.begin += size;
				Span.end += size;
			}
		};
		for (auto &place : nodes)
		{
			move(place.span);
		}
		for (auto &place : sites)
		{
			move(place.span);
		}
		for (auto &place : instances)
		{
			move(place.span);
		}
		if (init.valid())
		{
			move(init);
		}
		else if (Other.init.valid())
		{
			init = {Other.init.begin + Location, Other.init.end + Location};
		}
		text.insert(static_cast<std::size_t>(Location), Other.text);
		for (auto place : Other.nodes)
		{
			place.span.begin += Location;
			place.span.end += Location;
			nodes.emplace_back(place);
		}
		for (auto place : Other.sites)
		{
			place.span.begin += Location;
			place.span.end += Location;
			sites.emplace_back(place);
		}
		for (auto place : Other.instances)
		{
			place.span.begin += Location;
			place.span.end += Location;
			instances.emplace_back(place);
		}
	}
};

struct node_layers::save_state
{
	std::map<layer_handle, std::string> content; // original text of the files
	std::map<layer_handle, file_patch> patches;
	std::map<layer_handle, std::vector<basic_node const *>> nodes; // unchanged nodes defined in each file
	std::map<layer_handle, std::vector<layer_handle>> includes; // unchanged include directives in each file, by included layer
	std::map<layer_handle, std::vector<instance_handle>> instances; // unchanged includes of templates in each file
	std::string eol{"\r\n"};
	std::string error;
};

// produces text of specified file with the changes applied
bool node_layers::compose(save_state &State, layer_handle const Layer, composition &Output, int const Depth) const
{
	if (Depth > 32)
	{
		State.error = "layers include each other in a loop";
		return false;
	}
	static std::string const empty;
	auto const contentlookup{State.content.find(Layer)};
	auto const &base{contentlookup != State.content.end() ? contentlookup->second : empty};
	auto const basesize{static_cast<std::streamoff>(base.size())};
	auto const eol{detect_eol(base, State.eol)};

	std::vector<file_patch::edit> edits;
	if (auto const lookup{State.patches.find(Layer)}; lookup != State.patches.end())
	{
		edits = lookup->second.edits;
	}
	// what's added to the file goes ahead of the scenario initialization, if the file takes a part in it
	auto const &init{layer(Layer).init};
	auto const hasinit{init.valid() && init.end <= basesize};
	auto addition{basesize};
	if (hasinit)
	{
		addition = init.begin;
		// on a line of its own, where the initialization has one
		auto linebegin{addition};
		while (linebegin > 0 && (base[linebegin - 1] == ' ' || base[linebegin - 1] == '\t'))
		{
			--linebegin;
		}
		if (linebegin == 0 || base[linebegin - 1] == '\n')
		{
			addition = linebegin;
		}
	}
	auto initedited{false}; // the directive standing for the initialization is replaced or removed
	for (auto &edit : edits)
	{
		if (edit.begin < 0)
		{
			edit.begin = edit.end = addition;
		}
		initedited |= hasinit && edit.begin == init.begin && edit.end == init.end;
	}
	std::stable_sort(std::begin(edits), std::end(edits), [](file_patch::edit const &Left, file_patch::edit const &Right) { return Left.begin < Right.begin; });
	std::streamoff cursor{0};
	for (auto const &edit : edits)
	{
		if (edit.begin < cursor || edit.end < edit.begin || edit.end > basesize)
		{
			State.error = "conflicting changes in file \"" + layer(Layer).name + "\"";
			return false;
		}
		cursor = edit.end;
	}

	// locations in the original text past which everything is shifted by given amount
	std::vector<std::pair<std::streamoff, std::streamoff>> shifts;
	cursor = 0;
	for (std::size_t idx = 0; idx < edits.size(); ++idx)
	{
		auto const &edit{edits[idx]};
		auto begin{edit.begin};
		auto end{edit.end};
		if (edit.text.empty() && edit.inlined == null_handle && end > begin)
		{
			// a definition is removed. if it had a line for itself take the whole line out, instead of leaving it blank
			auto const limit{idx + 1 < edits.size() ? edits[idx + 1].begin : basesize};
			auto linebegin{begin};
			while (linebegin > cursor && (base[linebegin - 1] == ' ' || base[linebegin - 1] == '\t'))
			{
				--linebegin;
			}
			auto lineend{end};
			while (lineend < limit && (base[lineend] == ' ' || base[lineend] == '\t'))
			{
				++lineend;
			}
			auto const startsline{linebegin == 0 || base[linebegin - 1] == '\n'};
			auto endsline{lineend == basesize};
			if (lineend + 1 < limit && base[lineend] == '\r' && base[lineend + 1] == '\n')
			{
				lineend += 2;
				endsline = true;
			}
			else if (lineend < limit && base[lineend] == '\n')
			{
				lineend += 1;
				endsline = true;
			}
			if (startsline && endsline)
			{
				begin = linebegin;
				end = lineend;
			}
		}
		Output.text.append(base, static_cast<std::size_t>(cursor), static_cast<std::size_t>(begin - cursor));
		if (edit.inlined != null_handle)
		{
			// content of the included file takes place of the include directive
			composition included;
			if (false == compose(State, edit.inlined, included, Depth + 1))
			{
				return false;
			}
			if (end < basesize && (base[end] == '\r' || base[end] == '\n'))
			{
				// the line break which followed the directive ends the included text now, a second one would leave a blank line
				if (false == included.text.empty() && included.text.back() == '\n')
				{
					included.text.pop_back();
					if (false == included.text.empty() && included.text.back() == '\r')
					{
						included.text.pop_back();
					}
				}
			}
			else
			{
				ensure_newline(included.text, eol);
			}
			Output.append(included);
		}
		else
		{
			// the line break leading the replacement isn't needed if the text ahead happens to end with one
			auto const lead{(Output.text.empty() || Output.text.back() == '\n') ? std::size_t{0} : edit.lead};
			auto const offset{static_cast<std::streamoff>(Output.text.size() + lead)};
			auto const span{source_span{offset, offset + static_cast<std::streamoff>(edit.length)}};
			if (edit.node != nullptr)
			{
				Output.nodes.push_back({edit.node, span, true});
			}
			if (edit.site != null_handle)
			{
				Output.sites.push_back({edit.site, span});
			}
			if (edit.instance != 0)
			{
				Output.instances.push_back({edit.instance, span});
			}
			Output.text.append(edit.text, edit.lead - lead, std::string::npos);
		}
		cursor = end;
		shifts.emplace_back(end, static_cast<std::streamoff>(Output.text.size()) - end);
	}
	Output.text.append(base, static_cast<std::size_t>(cursor), std::string::npos);

	// what the changes didn't touch only moves along with the text around it
	auto const shift = [&](source_span Span) {
		auto const lookup{std::upper_bound(std::begin(shifts), std::end(shifts), Span.begin,
		                                   [](std::streamoff const Location, std::pair<std::streamoff, std::streamoff> const &Shift) { return Location < Shift.first; })};
		auto const offset{lookup == std::begin(shifts) ? 0 : std::prev(lookup)->second};
		Span.begin += offset;
		Span.end += offset;
		return Span;
	};
	if (auto const lookup{State.nodes.find(Layer)}; lookup != State.nodes.end())
	{
		for (auto const *node : lookup->second)
		{
			Output.nodes.push_back({node, shift(m_sources.at(node).span), false});
		}
	}
	if (auto const lookup{State.includes.find(Layer)}; lookup != State.includes.end())
	{
		for (auto const included : lookup->second)
		{
			Output.sites.push_back({included, shift(layer(included).sites.front().span)});
		}
	}
	if (auto const lookup{State.instances.find(Layer)}; lookup != State.instances.end())
	{
		for (auto const included : lookup->second)
		{
			Output.instances.push_back({included, shift(instance(included).span)});
		}
	}
	// NOTE: with the file holding the initialization inlined, its own mark was carried over along with its text
	if (hasinit && false == initedited && false == Output.init.valid())
	{
		Output.init = shift(init);
	}
	return true;
}

// writes changes made in the editor to the scenery files
save_result node_layers::save(std::vector<std::string> const &Rootstatements)
{
	save_result result;
	if (m_layers.empty())
	{
		result.message = "The scenery wasn't opened for editing, there are no layers to save.";
		return result;
	}
	auto const root{layer_handle{1}};
	save_state state;
	auto const fail = [&](std::string const &Message) {
		result.message = "Scenery not saved: " + Message;
		ErrorLog(result.message);
		return result;
	};
	// loads text of specified layer file, if it wasn't loaded yet
	auto const load = [&](layer_handle const Layer) {
		if (layer(Layer).created || state.content.count(Layer) != 0)
		{
			return true;
		}
		std::error_code error;
		auto const filepath{std::filesystem::path(path(Layer))};
		auto const filesize{std::filesystem::file_size(filepath, error)};
		auto const filetime{error ? std::filesystem::file_time_type{} : std::filesystem::last_write_time(filepath, error)};
		std::string content;
		if (error || false == read_file(path(Layer), content))
		{
			state.error = "can't read file \"" + layer(Layer).name + "\"";
			return false;
		}
		if (filesize != layer(Layer).filesize || filetime != layer(Layer).filetime || content.size() != filesize)
		{
			// locations of the definitions are no longer valid
			state.error = "file \"" + layer(Layer).name + "\" was changed outside of the editor since the scenery was loaded";
			return false;
		}
		state.content.emplace(Layer, std::move(content));
		return true;
	};
	// a layer is saved if it's still a part of the scenery, as a file of its own
	auto const is_output = [&](layer_handle const Layer) { return listed(Layer) && false == layer(Layer).removed && false == layer(Layer).binary; };

	if (false == load(root))
	{
		return fail(state.error);
	}
	state.eol = detect_eol(state.content[root], "\r\n");

	// changed and deleted nodes
	std::set<basic_node const *> rewritten;
	std::map<layer_handle, std::vector<std::pair<basic_node const *, std::string>>> created; // new definitions, by target layer
	auto const process = [&](basic_node const *Node, TAnimModel const *Model) {
		auto const nodelayer{resolve(Node->layer())};
		if (false == is_output(nodelayer))
		{
			return true;
		}
		auto const location{Node->location()};
		auto const angles{Model != nullptr ? Model->Angles() : glm::vec3{0.f}};
		auto const scale{Model != nullptr ? Model->Scale() : glm::vec3{1.f}};
		auto const lookup{m_sources.find(Node)};
		std::string error;
		if (lookup != m_sources.end())
		{
			// defined in a scenery file. rewrite the placement in its definition if it's no longer the one it was loaded with
			auto const &source{lookup->second};
			if (false == writable(source.layer))
			{
				// the editor doesn't let these be changed, as one definition stands for more than one node
				return true;
			}
			auto const moved{glm::distance(location, source.location) > 1e-4};
			auto const rotated{false == glm::all(glm::epsilonEqual(angles, source.angles, 1e-3f))};
			auto const scaled{false == glm::all(glm::epsilonEqual(scale, source.scale, 1e-4f))};
			if (false == (moved || rotated || scaled))
			{
				return true;
			}
			if (false == load(source.layer))
			{
				return false;
			}
			auto const &content{state.content[source.layer]};
			if (source.span.end > static_cast<std::streamoff>(content.size()))
			{
				state.error = "definition of \"" + Node->name() + "\" is out of bounds of file \"" + layer(source.layer).name + "\"";
				return false;
			}
			auto text{content.substr(static_cast<std::size_t>(source.span.begin), static_cast<std::size_t>(source.span.end - source.span.begin))};
			auto patched{false};
			if (Model != nullptr)
			{
				model_placement placement;
				placement.rotation = source.context.rotation;
				if (moved)
				{
					placement.location = source.context.to_local(location);
				}
				if (rotated)
				{
					placement.angles = angles;
				}
				if (scaled)
				{
					placement.scale = scale / source.context.scale;
				}
				patched = patch_model(text, placement, error);
			}
			else
			{
				patched = patch_memcell(text, source.context.to_local(location), error);
			}
			if (false == patched)
			{
				state.error = error + " of \"" + Node->name() + "\" in file \"" + layer(source.layer).name + "\"";
				return false;
			}
			file_patch::edit edit;
			edit.begin = source.span.begin;
			edit.end = source.span.end;
			edit.length = text.size();
			edit.text = std::move(text);
			edit.node = Node;
			state.patches[source.layer].edits.emplace_back(std::move(edit));
			rewritten.emplace(Node);
		}
		else if (Model != nullptr && false == Node->from_template())
		{
			// created in the editor. the definition is added to the layer file, with the placement in effect at the place it goes to
			std::string text;
			Node->export_as_text(text);
			text.erase(text.find_last_not_of(" \t\r\n") + 1);
			auto const &context{layer(nodelayer).context_insert()};
			if (false == context.matches(layer_context()))
			{
				model_placement placement;
				placement.rotation = context.rotation;
				placement.location = context.to_local(location);
				placement.angles = angles;
				placement.scale = scale / context.scale;
				if (false == patch_model(text, placement, error))
				{
					state.error = error + " of \"" + Node->name() + "\"";
					return false;
				}
			}
			created[nodelayer].emplace_back(Node, std::move(text));
		}
		return true;
	};
	for (auto const *instance : simulation::Instances.sequence())
	{
		if (instance != nullptr && false == process(instance, instance))
		{
			return fail(state.error);
		}
	}
	for (auto const *memorycell : simulation::Memory.sequence())
	{
		if (memorycell != nullptr && false == process(memorycell, nullptr))
		{
			return fail(state.error);
		}
	}
	auto const push_edit = [&](layer_handle const Layer, source_span const &Span, std::string Text, basic_node const *Node) {
		file_patch::edit edit;
		edit.begin = Span.begin;
		edit.end = Span.end;
		edit.length = Text.size();
		edit.text = std::move(Text);
		edit.node = Node;
		state.patches[Layer].edits.emplace_back(std::move(edit));
	};
	// traction and event launchers are rewritten when the editor moved them along with the track they belong to
	std::vector<basic_node *> patchednodes;
	auto const patch_node = [&](basic_node *Node, auto const &Patch) {
		auto const lookup{m_sources.find(Node)};
		if (lookup == m_sources.end() || false == Node->dirty())
		{
			return true;
		}
		auto const &source{lookup->second};
		if (false == is_output(resolve(source.layer)) || false == writable(source.layer))
		{
			return true;
		}
		if (false == load(source.layer))
		{
			return false;
		}
		auto const &content{state.content[source.layer]};
		if (source.span.end > static_cast<std::streamoff>(content.size()))
		{
			state.error = "definition of \"" + Node->name() + "\" is out of bounds of file \"" + layer(source.layer).name + "\"";
			return false;
		}
		auto text{content.substr(static_cast<std::size_t>(source.span.begin), static_cast<std::size_t>(source.span.end - source.span.begin))};
		std::string error;
		if (false == Patch(text, source, error))
		{
			state.error = error + " of \"" + Node->name() + "\" in file \"" + layer(source.layer).name + "\"";
			return false;
		}
		push_edit(source.layer, source.span, std::move(text), Node);
		rewritten.emplace(Node);
		patchednodes.push_back(Node);
		return true;
	};
	for (auto *traction : simulation::Traction.sequence())
	{
		if (traction != nullptr &&
		    false == patch_node(traction, [&](std::string &Text, node_source const &Source, std::string &Error) { return patch_traction(Text, *traction, Source.context.offset, Error); }))
		{
			return fail(state.error);
		}
	}
	for (auto *launcher : simulation::Events.launchers())
	{
		if (launcher != nullptr &&
		    false == patch_node(launcher, [&](std::string &Text, node_source const &Source, std::string &Error) { return patch_launcher(Text, Source.context.to_local(launcher->location()), Error); }))
		{
			return fail(state.error);
		}
	}
	std::vector<TTrack *> savedpaths;
	std::vector<TTrack const *> droppedpaths;
	auto const path_text = [](TTrack &Path, glm::dvec3 const &Offset) {
		auto const paths{Path.m_paths};
		for (auto &path : Path.m_paths)
		{
			path.points[segment_data::point::start] -= Offset;
			path.points[segment_data::point::end] -= Offset;
		}
		std::string text;
		Path.export_as_text(text);
		Path.m_paths = paths;
		text.erase(text.find_last_not_of(" \t\r\n") + 1);
		return text;
	};
	for (auto *path : simulation::Paths.sequence())
	{
		if (path == nullptr || (path->iCategoryFlag & 0x80) != 0 || path->m_road != nullptr || path->from_template())
		{
			continue;
		}
		auto const nodelayer{resolve(path->layer())};
		if (false == is_output(nodelayer))
		{
			continue;
		}
		auto const lookup{m_sources.find(path)};
		if (lookup == m_sources.end())
		{
			if (path->m_editorremoved || false == path->dirty())
			{
				continue;
			}
			created[nodelayer].emplace_back(path, path_text(*path, layer(nodelayer).context_insert().offset));
			savedpaths.push_back(path);
			continue;
		}
		auto const &source{lookup->second};
		if ((false == path->m_editorremoved && false == path->dirty()) || false == writable(source.layer))
		{
			continue;
		}
		if (false == load(source.layer))
		{
			return fail(state.error);
		}
		if (path->m_editorremoved)
		{
			push_edit(source.layer, source.span, {}, nullptr);
			droppedpaths.push_back(path);
		}
		else
		{
			push_edit(source.layer, source.span, path_text(*path, source.context.offset), path);
			savedpaths.push_back(path);
		}
		rewritten.emplace(path);
	}
	// roads. the lanes aren't saved, they're generated from the definition of the road
	std::vector<road_node *> savedroads;
	std::vector<road_node const *> droppedroads;
	auto const road_text = [](road_node &Road, glm::dvec3 const &Offset) {
		auto const definition{Road.definition()};
		auto local{definition};
		local.axis.points[segment_data::point::start] -= Offset;
		local.axis.points[segment_data::point::end] -= Offset;
		Road.define(local);
		std::string text;
		Road.export_as_text(text);
		Road.define(definition);
		text.erase(text.find_last_not_of(" \t\r\n") + 1);
		return text;
	};
	for (auto *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road->from_template())
		{
			continue;
		}
		auto const nodelayer{resolve(road->layer())};
		if (false == is_output(nodelayer))
		{
			continue;
		}
		auto const lookup{m_sources.find(road)};
		if (lookup == m_sources.end())
		{
			if (road->m_editorremoved || false == road->dirty())
			{
				continue;
			}
			created[nodelayer].emplace_back(road, road_text(*road, layer(nodelayer).context_insert().offset));
			savedroads.push_back(road);
			continue;
		}
		auto const &source{lookup->second};
		if ((false == road->m_editorremoved && false == road->dirty()) || false == writable(source.layer))
		{
			continue;
		}
		if (false == load(source.layer))
		{
			return fail(state.error);
		}
		file_patch::edit edit;
		edit.begin = source.span.begin;
		edit.end = source.span.end;
		if (road->m_editorremoved)
		{
			droppedroads.push_back(road);
		}
		else
		{
			edit.text = road_text(*road, source.context.offset);
			edit.length = edit.text.size();
			edit.node = road;
			savedroads.push_back(road);
		}
		state.patches[source.layer].edits.emplace_back(std::move(edit));
		rewritten.emplace(road);
	}
	// road junctions, likewise
	std::vector<junction_node *> savedjunctions;
	std::vector<junction_node const *> droppedjunctions;
	auto const junction_text = [](junction_node &Junction, glm::dvec3 const &Offset) {
		auto const definition{Junction.definition()};
		auto local{definition};
		local.centre -= Offset;
		for (auto &arm : local.arms)
		{
			arm.position -= Offset;
		}
		Junction.define(local);
		std::string text;
		Junction.export_as_text(text);
		Junction.define(definition);
		text.erase(text.find_last_not_of(" \t\r\n") + 1);
		return text;
	};
	for (auto *junction : simulation::Junctions.sequence())
	{
		if (junction == nullptr || junction->from_template())
		{
			continue;
		}
		auto const nodelayer{resolve(junction->layer())};
		if (false == is_output(nodelayer))
		{
			continue;
		}
		auto const lookup{m_sources.find(junction)};
		if (lookup == m_sources.end())
		{
			if (junction->m_editorremoved || false == junction->dirty())
			{
				continue;
			}
			created[nodelayer].emplace_back(junction, junction_text(*junction, layer(nodelayer).context_insert().offset));
			savedjunctions.push_back(junction);
			continue;
		}
		auto const &source{lookup->second};
		if ((false == junction->m_editorremoved && false == junction->dirty()) || false == writable(source.layer))
		{
			continue;
		}
		if (false == load(source.layer))
		{
			return fail(state.error);
		}
		file_patch::edit edit;
		edit.begin = source.span.begin;
		edit.end = source.span.end;
		if (junction->m_editorremoved)
		{
			droppedjunctions.push_back(junction);
		}
		else
		{
			edit.text = junction_text(*junction, source.context.offset);
			edit.length = edit.text.size();
			edit.node = junction;
			savedjunctions.push_back(junction);
		}
		state.patches[source.layer].edits.emplace_back(std::move(edit));
		rewritten.emplace(junction);
	}
	// level crossings and traffic points of the roads
	std::vector<roadpoint_node *> savedroadpoints;
	std::vector<roadpoint_node const *> droppedroadpoints;
	auto const roadpoint_text = [](roadpoint_node &Point, glm::dvec3 const &Offset) {
		auto const definition{Point.definition()};
		auto local{definition};
		local.position -= Offset;
		Point.define(local);
		std::string text;
		Point.export_as_text(text);
		Point.define(definition);
		text.erase(text.find_last_not_of(" \t\r\n") + 1);
		return text;
	};
	for (auto *point : simulation::Roadpoints.sequence())
	{
		if (point == nullptr || point->from_template())
		{
			continue;
		}
		auto const nodelayer{resolve(point->layer())};
		if (false == is_output(nodelayer))
		{
			continue;
		}
		auto const lookup{m_sources.find(point)};
		if (lookup == m_sources.end())
		{
			if (point->m_editorremoved || false == point->dirty())
			{
				continue;
			}
			created[nodelayer].emplace_back(point, roadpoint_text(*point, layer(nodelayer).context_insert().offset));
			savedroadpoints.push_back(point);
			continue;
		}
		auto const &source{lookup->second};
		if ((false == point->m_editorremoved && false == point->dirty()) || false == writable(source.layer))
		{
			continue;
		}
		if (false == load(source.layer))
		{
			return fail(state.error);
		}
		file_patch::edit edit;
		edit.begin = source.span.begin;
		edit.end = source.span.end;
		if (point->m_editorremoved)
		{
			droppedroadpoints.push_back(point);
		}
		else
		{
			edit.text = roadpoint_text(*point, source.context.offset);
			edit.length = edit.text.size();
			edit.node = point;
			savedroadpoints.push_back(point);
		}
		state.patches[source.layer].edits.emplace_back(std::move(edit));
		rewritten.emplace(point);
	}
	for (auto const &erased : m_erased)
	{
		file_patch::edit edit;
		edit.begin = erased.second.begin;
		edit.end = erased.second.end;
		state.patches[erased.first].edits.emplace_back(std::move(edit));
	}

	// editor data in the comment lines: the old lines go, the new ones are added to the file like new definitions
	std::map<layer_handle, std::vector<std::string>> markedtext;
	for (auto const &marking : m_marking)
	{
		auto const markedlayer{marking.first.first};
		auto const &mark{marking.first.second};
		if (auto const lookup{m_marked.find(markedlayer)}; lookup != m_marked.end())
		{
			for (auto const &line : lookup->second)
			{
				if (line.mark == mark)
				{
					push_edit(markedlayer, line.span, {}, nullptr);
				}
			}
		}
		auto const target{resolve(markedlayer)};
		if (marking.second.empty() || false == is_output(target))
		{
			continue;
		}
		for (auto const &text : marking.second)
		{
			markedtext[target].push_back(mark + ' ' + text);
		}
	}

	// includes of templates
	std::map<layer_handle, std::vector<instance_handle>> placed; // directives made in the editor, by target layer
	for (std::size_t idx = 0; idx < m_instances.size(); ++idx)
	{
		auto const &included{m_instances[idx]};
		if (included.dead)
		{
			continue;
		}
		if (included.span.valid())
		{
			if (included.removed)
			{
				file_patch::edit edit;
				edit.begin = included.span.begin;
				edit.end = included.span.end;
				state.patches[included.layer].edits.emplace_back(std::move(edit));
			}
			else if (false == included.directive.empty())
			{
				// changed in the editor, the new directive takes the place of the one in the file
				file_patch::edit edit;
				edit.begin = included.span.begin;
				edit.end = included.span.end;
				edit.length = included.directive.size();
				edit.text = included.directive;
				edit.instance = static_cast<instance_handle>(idx + 1);
				state.patches[included.layer].edits.emplace_back(std::move(edit));
			}
		}
		else if (false == included.removed && false == included.directive.empty())
		{
			auto const target{resolve(included.layer)};
			if (false == is_output(target))
			{
				continue;
			}
			if (false == included.context.matches(layer(target).context_insert()))
			{
				// shouldn't happen, merge of layers is refused when it'd lead to this
				return fail("include \"" + *included.file + "\" was placed for different origin, rotation or scale than its layer file receives it with");
			}
			placed[target].emplace_back(static_cast<instance_handle>(idx + 1));
		}
	}

	// layer changes
	std::map<layer_handle, std::vector<layer_handle>> appended; // merged layers placed at the end of their target
	std::set<layer_handle> absorbed; // merged layers which cease to exist as files
	std::set<layer_handle> detached; // layers whose include directives are replaced
	for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
	{
		auto const candidate{static_cast<layer_handle>(idx + 1)};
		auto const &source{layer(candidate)};
		if (source.dead)
		{
			continue;
		}
		auto const hassite{false == source.created && source.sites.size() == 1 && false == source.sites.front().fixed && source.sites.front().span.valid()};
		if (source.merged != null_handle)
		{
			auto const target{resolve(candidate)};
			absorbed.emplace(candidate);
			if (hassite)
			{
				file_patch::edit edit;
				edit.begin = source.sites.front().span.begin;
				edit.end = source.sites.front().span.end;
				if (resolve(source.sites.front().parent) == target)
				{
					edit.inlined = candidate;
				}
				else
				{
					appended[target].emplace_back(candidate);
				}
				state.patches[source.sites.front().parent].edits.emplace_back(std::move(edit));
				detached.emplace(candidate);
			}
			else
			{
				appended[target].emplace_back(candidate);
			}
		}
		else if (source.removed)
		{
			if (hassite)
			{
				file_patch::edit edit;
				edit.begin = source.sites.front().span.begin;
				edit.end = source.sites.front().span.end;
				state.patches[source.sites.front().parent].edits.emplace_back(std::move(edit));
				detached.emplace(candidate);
			}
		}
		else if (source.created)
		{
			// include directive for the new file, on a line of its own
			auto const parent{source.parent};
			auto const directive{"include " + source.name + " end"};
			file_patch::edit edit;
			edit.site = candidate;
			edit.length = directive.size();
			if (valid(source.anchor) && layer(source.anchor).sites.size() == 1 && layer(source.anchor).sites.front().parent == parent && layer(source.anchor).sites.front().span.valid())
			{
				edit.begin = edit.end = layer(source.anchor).sites.front().span.end;
				edit.text = state.eol + directive;
				edit.lead = state.eol.size();
			}
			else
			{
				// NOTE: negative location stands for the place the file receives additions at
				edit.begin = edit.end = -1;
				if (false == layer(parent).created)
				{
					if (false == load(parent))
					{
						return fail(state.error);
					}
					// dropped by compose() if the text ahead ends with a line break already
					edit.text = state.eol;
					edit.lead = state.eol.size();
				}
				edit.text += directive + state.eol;
			}
			state.patches[parent].edits.emplace_back(std::move(edit));
		}
	}

	// files to write
	std::vector<layer_handle> outputs;
	for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
	{
		auto const candidate{static_cast<layer_handle>(idx + 1)};
		if (false == is_output(candidate))
		{
			continue;
		}
		if (layer(candidate).created || state.patches.count(candidate) != 0 || appended.count(candidate) != 0 || created.count(candidate) != 0 || placed.count(candidate) != 0 ||
		    markedtext.count(candidate) != 0 || (candidate == root && false == Rootstatements.empty()))
		{
			outputs.emplace_back(candidate);
		}
	}
	// text of the merged layers travels with the file they end up in, but changes made to it are still theirs
	std::vector<layer_handle> composed{outputs};
	composed.insert(std::end(composed), std::begin(absorbed), std::end(absorbed));
	for (auto const candidate : composed)
	{
		if (false == load(candidate))
		{
			return fail(state.error);
		}
	}
	// what is defined in these files and isn't rewritten moves along with the text
	for (auto const &source : m_sources)
	{
		if (state.content.count(source.second.layer) != 0 && rewritten.count(source.first) == 0)
		{
			state.nodes[source.second.layer].emplace_back(source.first);
		}
	}
	for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
	{
		auto const candidate{static_cast<layer_handle>(idx + 1)};
		auto const &included{layer(candidate)};
		if (included.dead || included.created || detached.count(candidate) != 0 || included.sites.size() != 1 || included.sites.front().fixed || false == included.sites.front().span.valid())
		{
			continue;
		}
		state.includes[included.sites.front().parent].emplace_back(candidate);
	}
	for (std::size_t idx = 0; idx < m_instances.size(); ++idx)
	{
		auto const &included{m_instances[idx]};
		if (false == included.dead && false == included.removed && included.directive.empty() && included.span.valid() && state.content.count(included.layer) != 0)
		{
			state.instances[included.layer].emplace_back(static_cast<instance_handle>(idx + 1));
		}
	}

	std::map<layer_handle, composition> texts;
	for (auto const output : outputs)
	{
		auto &text{texts[output]};
		auto const eol{detect_eol(state.content.count(output) != 0 ? state.content[output] : std::string{}, state.eol)};
		if (layer(output).created)
		{
			// NOTE: the parser treats an empty file as a missing one
			text.text = "// scenery layer created in the editor" + eol;
		}
		composition body;
		if (false == compose(state, output, body, 0))
		{
			return fail(state.error);
		}
		text.append(body);
		// what the editor adds to the file
		composition added;
		if (auto const lookup{appended.find(output)}; lookup != appended.end())
		{
			for (auto const merged : lookup->second)
			{
				composition mergedtext;
				if (false == compose(state, merged, mergedtext, 0))
				{
					return fail(state.error);
				}
				ensure_newline(added.text, eol);
				added.append(mergedtext);
			}
		}
		if (auto const lookup{created.find(output)}; lookup != created.end())
		{
			for (auto const &definition : lookup->second)
			{
				ensure_newline(added.text, eol);
				auto const offset{static_cast<std::streamoff>(added.text.size())};
				added.nodes.push_back({definition.first, {offset, offset + static_cast<std::streamoff>(definition.second.size())}, true});
				added.text += definition.second + eol;
			}
		}
		if (auto const lookup{placed.find(output)}; lookup != placed.end())
		{
			for (auto const included : lookup->second)
			{
				auto const &directive{instance(included).directive};
				ensure_newline(added.text, eol);
				auto const offset{static_cast<std::streamoff>(added.text.size())};
				added.instances.push_back({included, {offset, offset + static_cast<std::streamoff>(directive.size())}});
				added.text += directive + eol;
			}
		}
		if (auto const lookup{markedtext.find(output)}; lookup != markedtext.end())
		{
			for (auto const &line : lookup->second)
			{
				ensure_newline(added.text, eol);
				added.text += line + eol;
			}
		}
		if (output == root)
		{
			for (auto const &statement : Rootstatements)
			{
				ensure_newline(added.text, eol);
				added.text += statement + eol;
			}
		}
		if (added.text.empty())
		{
			continue;
		}
		if (false == text.init.valid())
		{
			ensure_newline(text.text, eol);
			text.append(added);
			continue;
		}
		// the scenario initialization is meant to be followed by nothing but the vehicles, so the additions go ahead
		// of it. they take whole lines, with the initialization left at the start of a line the way it was
		ensure_newline(added.text, eol);
		auto location{text.init.begin};
		while (location > 0 && (text.text[location - 1] == ' ' || text.text[location - 1] == '\t'))
		{
			--location;
		}
		if (location == 0 || text.text[location - 1] == '\n')
		{
			text.insert(location, added);
		}
		else
		{
			composition separated;
			separated.text = eol;
			separated.append(added);
			text.insert(text.init.begin, separated);
		}
	}

	// with everything worked out it's safe to touch the files. new content goes to temporary files first,
	// so a failure half way through doesn't leave the scenery with only a part of the changes
	std::error_code error;
	std::vector<layer_handle> written;
	for (auto const output : outputs)
	{
		auto const lookup{state.content.find(output)};
		if (lookup != state.content.end() && lookup->second == texts[output].text)
		{
			continue;
		}
		auto const filepath{std::filesystem::path(path(output))};
		if (filepath.has_parent_path())
		{
			std::filesystem::create_directories(filepath.parent_path(), error);
		}
		std::ofstream file{path(output) + ".tmp", std::ios_base::binary | std::ios_base::trunc};
		file << texts[output].text;
		file.close();
		if (file.fail())
		{
			for (auto const leftover : written)
			{
				std::filesystem::remove(path(leftover) + ".tmp", error);
			}
			std::filesystem::remove(path(output) + ".tmp", error);
			return fail("can't write file \"" + layer(output).name + "\"");
		}
		written.emplace_back(output);
	}
	std::vector<layer_handle> replaced;
	for (auto const output : written)
	{
		auto const filepath{path(output)};
		// the first save of a file leaves its original content next to it
		if (false == layer(output).editormade && std::filesystem::exists(filepath, error) && false == std::filesystem::exists(filepath + ".bak", error))
		{
			std::filesystem::copy_file(filepath, filepath + ".bak", error);
		}
		std::filesystem::rename(filepath + ".tmp", filepath, error);
		if (error)
		{
			// e.g. the file is held open by another program. take back what was done so far: the scenery files
			// have to stay in step with each other, and with what we know about them
			auto const cause{error.message()};
			for (auto const restored : replaced)
			{
				auto const lookup{state.content.find(restored)};
				if (lookup != state.content.end())
				{
					std::ofstream{path(restored), std::ios_base::binary | std::ios_base::trunc} << lookup->second;
					stat(restored);
				}
				else
				{
					std::filesystem::remove(path(restored), error);
				}
			}
			for (auto const leftover : written)
			{
				std::filesystem::remove(path(leftover) + ".tmp", error);
			}
			result.files.clear();
			return fail("can't replace file \"" + layer(output).name + "\": " + cause);
		}
		replaced.emplace_back(output);
		result.files.emplace_back(layer(output).name);
	}
	for (auto const &file : result.files)
	{
		WriteLog("Scenery save: file \"" + file + "\" updated");
	}

	// the scenery files now match the scene, update the bookkeeping to reflect it
	auto removedterrain{false};
	for (auto const output : outputs)
	{
		for (auto const &place : texts[output].nodes)
		{
			auto &source{m_sources[place.node]};
			auto const isnew{source.layer == null_handle};
			source.layer = output;
			source.span = place.span;
			if (isnew)
			{
				source.context = layer(output).context_insert();
			}
			if (isnew || place.rewritten)
			{
				auto const *model{dynamic_cast<TAnimModel const *>(place.node)};
				source.location = place.node->location();
				source.angles = (model != nullptr ? model->Angles() : glm::vec3{0.f});
				source.scale = (model != nullptr ? model->Scale() : glm::vec3{1.f});
			}
		}
		for (auto const &place : texts[output].instances)
		{
			auto &included{m_instances[place.instance - 1]};
			included.layer = output;
			included.span = place.span;
			included.directive.clear();
		}
		for (auto const &place : texts[output].sites)
		{
			auto &included{layer(place.layer)};
			if (included.sites.empty())
			{
				// directive of a layer created in the editor
				included.sites.emplace_back();
			}
			included.sites.front().parent = output;
			included.sites.front().span = place.span;
			included.parent = output;
		}
		// the mark of the scenario initialization moved along with the text around it, or came with the text of a merged layer
		auto &saved{layer(output)};
		if (false == saved.init.valid() && texts[output].init.valid())
		{
			for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
			{
				if (m_layers[idx].init.valid() && m_layers[idx].merged != null_handle && resolve(static_cast<layer_handle>(idx + 1)) == output)
				{
					saved.context_init = m_layers[idx].context_init;
				}
			}
		}
		saved.init = texts[output].init;
	}
	for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
	{
		auto const candidate{static_cast<layer_handle>(idx + 1)};
		auto &saved{layer(candidate)};
		if (saved.dead)
		{
			continue;
		}
		if (saved.merged != null_handle || saved.removed)
		{
			removedterrain |= saved.removed && (saved.items[static_cast<std::size_t>(layer_item::shape)] + saved.items[static_cast<std::size_t>(layer_item::lines)] > 0);
			saved.dead = true;
			saved.sites.clear();
		}
		saved.created = false;
		saved.anchor = null_handle;
	}
	for (auto const output : written)
	{
		stat(output);
		scan(output);
	}
	for (auto const &marking : m_marking)
	{
		if (resolve(marking.first.first) != marking.first.first)
		{
			// the lines of the merged layer are now in the file of its target
			m_marked.erase(marking.first.first);
		}
	}
	m_marking.clear();
	for (auto &included : m_instances)
	{
		// removed includes are gone from the files now
		included.dead = included.dead || included.removed;
	}
	m_erased.clear();
	for (auto *path : savedpaths)
	{
		path->m_dirty = false;
	}
	for (auto *node : patchednodes)
	{
		node->m_dirty = false;
	}
	for (auto const *path : droppedpaths)
	{
		m_sources.erase(path);
	}
	for (auto *road : savedroads)
	{
		road->m_dirty = false;
	}
	for (auto const *road : droppedroads)
	{
		m_sources.erase(road);
	}
	for (auto *junction : savedjunctions)
	{
		junction->m_dirty = false;
	}
	for (auto const *junction : droppedjunctions)
	{
		m_sources.erase(junction);
	}
	for (auto *point : savedroadpoints)
	{
		point->m_dirty = false;
	}
	for (auto const *point : droppedroadpoints)
	{
		m_sources.erase(point);
	}
	if (removedterrain)
	{
		// binary terrain file still holds geometry of the dropped layer, have it rebuilt on the next load
		auto terrainfile{layer(root).name};
		while (false == terrainfile.empty() && terrainfile.front() == '$')
		{
			terrainfile.erase(0, 1);
		}
		erase_extension(terrainfile);
		if (std::filesystem::remove(Global.asCurrentSceneryPath + terrainfile + ".sbt", error))
		{
			WriteLog("Scenery save: binary terrain file removed, it'll be generated again on the next load");
		}
	}

	result.success = true;
	if (result.files.empty())
	{
		result.message = "No changes to save.";
	}
	else
	{
		result.message = "Saved: ";
		for (auto const &file : result.files)
		{
			result.message += file + (&file != &result.files.back() ? ", " : "");
		}
	}
	return result;
}

} // namespace scene

//---------------------------------------------------------------------------
