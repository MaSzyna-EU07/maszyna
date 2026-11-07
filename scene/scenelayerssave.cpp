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
#include "world/Sweep.h"
#include "world/RoadPoint.h"

#include "simulation/simulation.h"
#include "model/AnimModel.h"
#include "scene/scenenodegroups.h"
#include "world/MemCell.h"
#include "world/Traction.h"
#include "world/EvLaunch.h"
#include "world/Event.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "utilities/translation.h"
#include "editor/editorFormat.hpp"

#include <array>
#include <ctime>
#include <format>
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
	std::optional<std::string> name; // name of the node
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

	if (Placement.name)
	{
		replace(3, *Placement.name);
	}
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
bool patch_memcell(std::string &Text, std::optional<glm::dvec3> const &Location, std::string const &Track, std::string &Error)
{
	auto const tokens{tokenize(Text)};
	if (tokens.size() < 13 || tokens[0].text != "node" || tokens[4].text != "memcell" || tokens.back().text != "endmemcell")
	{
		Error = "unexpected layout of memory cell definition";
		return false;
	}
	std::vector<text_change> changes;
	if (Location)
		changes.insert(changes.end(), {{tokens[5].begin, tokens[5].end, number(Location->x)}, {tokens[6].begin, tokens[6].end, number(Location->y)}, {tokens[7].begin, tokens[7].end, number(Location->z)}});
	auto const &track{tokens[tokens.size() - 2]};
	if (false == Track.empty() && track.text != Track)
		changes.push_back({track.begin, track.end, Track});
	apply(Text, changes);
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

// how the placement of a model instance or memory cell differs from the one its definition was loaded with
struct placement_change
{
	glm::dvec3 location{0.0};
	bool moved{false};
	bool rotated{false};
	bool scaled{false};
	bool renamed{false};
	glm::vec3 angles{0.f};
	glm::vec3 scale{1.f};
	std::string track; // the track of a memory cell
};

// writes the changed placement into the definition text of a model instance, or of a memory cell
bool patch_placement(std::string &Text, std::string const &Name, bool const Model, node_source const &Source, placement_change const &Change, std::string &Error)
{
	if (false == Model)
	{
		return patch_memcell(Text, Change.moved ? std::optional<glm::dvec3>{Source.context.to_local(Change.location)} : std::nullopt, Change.track, Error);
	}
	model_placement placement;
	placement.rotation = Source.context.rotation;
	if (Change.moved)
	{
		placement.location = Source.context.to_local(Change.location);
	}
	if (Change.rotated)
	{
		placement.angles = Change.angles;
	}
	if (Change.scaled)
	{
		placement.scale = Change.scale / Source.context.scale;
	}
	if (Change.renamed)
	{
		placement.name = (Name.empty() ? "none" : Name);
	}
	return patch_model(Text, placement, Error);
}

// text of a track definition, in the coordinates of the file it goes to
std::string path_text(TTrack &Path, glm::dvec3 const &Offset)
{
	auto const paths{Path.m_paths};
	for (auto &path : Path.m_paths)
		path.translate(-Offset);
	std::string text;
	Path.export_as_text(text);
	Path.m_paths = paths;
	text.erase(text.find_last_not_of(" \t\r\n") + 1);
	return text;
}

// text of a road definition, likewise. the lanes aren't saved, they're generated from the definition of the road
std::string road_text(road_node &Road, glm::dvec3 const &Offset)
{
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
}

// text of a definition of models laid along a curve, likewise
std::string sweep_text(sweep_node &Sweep, glm::dvec3 const &Offset)
{
	auto const definition{Sweep.definition()};
	auto local{definition};
	for (auto &piece : local.pieces)
	{
		piece.points[segment_data::point::start] -= Offset;
		piece.points[segment_data::point::end] -= Offset;
	}
	Sweep.define(local);
	std::string text;
	Sweep.export_as_text(text);
	Sweep.define(definition);
	text.erase(text.find_last_not_of(" \t\r\n") + 1);
	return text;
}

// text of a road junction definition, likewise
std::string junction_text(junction_node &Junction, glm::dvec3 const &Offset)
{
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
}

// text of a level crossing or a traffic point of the roads, likewise
std::string roadpoint_text(roadpoint_node &Point, glm::dvec3 const &Offset)
{
	auto const definition{Point.definition()};
	auto local{definition};
	local.position -= Offset;
	Point.define(local);
	std::string text;
	Point.export_as_text(text);
	Point.define(definition);
	text.erase(text.find_last_not_of(" \t\r\n") + 1);
	return text;
}

// adds the lines at the end of the text, each on a line of its own
void append_lines(std::string &Text, std::vector<std::string> const &Lines, std::string const &Eol)
{
	for (auto const &line : Lines)
	{
		ensure_newline(Text, Eol);
		Text += line + Eol;
	}
}

// the message of a save which went through: the files it wrote, and the copies it kept of those which lost their ground
std::string saved_message(save_result const &Result)
{
	if (Result.files.empty())
	{
		return STR_C("No changes to save");
	}
	std::string files;
	for (auto const &file : Result.files)
	{
		files += file + (&file != &Result.files.back() ? ", " : "");
	}
	auto message{format(STR_C("Saved: %s"), files.c_str())};
	for (auto const &backup : Result.backups)
	{
		message += format(STR_C(". Backup before the ground was taken out: %s"), std::filesystem::path(backup).filename().string().c_str());
	}
	return message;
}

// the names of the first five nodes whose changes couldn't be written
std::string unwritten_names(std::vector<std::string> const &Unwritten)
{
	std::string names;
	for (std::size_t i = 0; i < std::min<std::size_t>(Unwritten.size(), 5); ++i)
	{
		names += (i > 0 ? ", " : "") + Unwritten[i];
	}
	if (Unwritten.size() > 5)
	{
		names += ", ...";
	}
	return names;
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
	// adds what the editor adds to the file: at the end, or ahead of the scenario initialization where the text holds it. that
	// one is meant to be followed by nothing but the vehicles, so the additions take whole lines ahead of it, with the
	// initialization left at the start of a line the way it was
	void add(composition &Added, std::string const &Eol)
	{
		if (false == init.valid())
		{
			ensure_newline(text, Eol);
			append(Added);
			return;
		}
		ensure_newline(Added.text, Eol);
		auto location{init.begin};
		while (location > 0 && (text[location - 1] == ' ' || text[location - 1] == '\t'))
		{
			--location;
		}
		if (location == 0 || text[location - 1] == '\n')
		{
			insert(location, Added);
		}
		else
		{
			composition separated;
			separated.text = Eol;
			separated.append(Added);
			insert(init.begin, separated);
		}
	}
};

struct node_layers::save_state
{
	explicit save_state(node_layers &Layers) : layers{Layers} {}

	node_layers &layers; // the layers being saved
	layer_handle const root{1}; // the scenario file
	std::map<layer_handle, std::string> content; // original text of the files
	std::map<layer_handle, file_patch> patches;
	std::map<layer_handle, std::vector<basic_node const *>> nodes; // unchanged nodes defined in each file
	std::map<layer_handle, std::vector<layer_handle>> includes; // unchanged include directives in each file, by included layer
	std::map<layer_handle, std::vector<instance_handle>> instances; // unchanged includes of templates in each file
	std::string eol{"\r\n"};
	std::string error;
	// what the save works out on its way to the files, see node_layers::save()
	using definitions = std::vector<std::pair<basic_node const *, std::string>>;
	std::set<basic_node const *> rewritten; // nodes whose definitions are written anew or taken out of their files
	std::map<layer_handle, definitions> created; // new definitions, by target layer
	std::set<basic_node const *> relocated; // models written anew along with their group, or out of a dissolved one
	std::vector<std::string> unwritten; // changes which can't be written, as their definition stands for more than one node
	std::vector<basic_node *> saved; // nodes whose current state the files receive
	std::vector<basic_node const *> dropped; // nodes taken out of the files
	std::map<layer_handle, std::vector<std::string>> markedtext; // editor data in the comment lines, by target layer
	std::map<layer_handle, std::vector<instance_handle>> placed; // directives made in the editor, by target layer
	std::map<layer_handle, std::vector<layer_handle>> appended; // merged layers placed at the end of their target
	std::set<layer_handle> absorbed; // merged layers which cease to exist as files
	std::set<layer_handle> detached; // layers whose include directives are replaced
	std::vector<layer_handle> outputs; // files to write
	std::vector<layer_handle> composed; // the files to write, with the merged layers whose text travels with them
	std::map<layer_handle, composition> texts; // new text of the files to write
	std::vector<layer_handle> written; // files whose new text waits in a temporary file

	bool load(layer_handle Layer);
	bool is_output(layer_handle Layer) const;
	void push_edit(layer_handle Layer, source_span const &Span, std::string Text, basic_node const *Node);
	// the changes in the scene, as changes to make in the files
	bool collect_changes();
	bool process_nodes();
	bool process(basic_node const *Node, TAnimModel const *Model);
	placement_change change_of(basic_node const *Node, TAnimModel const *Model, node_source const &Source) const;
	bool rewrite_placement(basic_node const *Node, TAnimModel const *Model, node_source const &Source);
	bool placed_definition(TAnimModel const *Model, layer_handle Target, std::string &Text);
	bool relocate_groups();
	bool relocate_group(group_handle Group);
	bool relocate(TAnimModel const *Model, layer_handle Target, definitions &Output);
	bool patch_attached();
	bool save_paths();
	bool save_path(TTrack *Path);
	void mark_lines();
	void replace_marked(layer_handle Markedlayer, std::string const &Mark, std::vector<std::string> const &Lines);
	void move_marked(layer_handle Markedlayer, std::vector<marked_line> const &Lines);
	bool place_templates();
	bool place_template(std::size_t Index);
	bool change_layers();
	bool change_layer(layer_handle Candidate);
	void merge_layer(layer_handle Candidate, bool Hassite);
	bool include_created(layer_handle Candidate);
	// the new text of the files
	void list_outputs(std::vector<std::string> const &Rootstatements, std::vector<std::string> const &Trailingstatements, std::map<layer_handle, std::vector<std::string>> const &Statements);
	bool load_composed();
	void track_unchanged();
	bool compose_outputs(std::vector<std::string> const &Rootstatements, std::vector<std::string> const &Trailingstatements, std::map<layer_handle, std::vector<std::string>> const &Statements);
	bool compose_output(layer_handle Output, std::vector<std::string> const &Rootstatements, std::vector<std::string> const &Trailingstatements, std::map<layer_handle, std::vector<std::string>> const &Statements);
	bool compose_merged(layer_handle Output, std::string const &Eol, composition &Added);
	void add_definitions(layer_handle Output, std::string const &Eol, composition &Added) const;
	// the files themselves
	bool write_temporary();
	bool write_temporary(layer_handle Output);
	void drop_temporary() const;
	bool back_up_ground(std::vector<std::string> &Backups);
	bool back_up(layer_handle Output, char const *Stamp, std::vector<std::string> &Backups);
	bool replace_files(save_result &Result);
	bool replace_file(layer_handle Output, std::vector<layer_handle> const &Replaced);
	void restore(std::vector<layer_handle> const &Replaced) const;
	// the bookkeeping, brought in line with the files
	void update_bookkeeping();
	void update_nodes(layer_handle Output);
	void update_includes(layer_handle Output);
	bool retire_layers();
	void forget_changes();
	void drop_terrain_cache(bool Removedterrain);

	// saves the nodes of one kind which are defined by what they hold: their definitions are written anew when they changed,
	// taken out of their files when removed, added to the files of their layers when they were created in the editor
	template <typename Sequence_, typename Text_>
	bool save_defined(Sequence_ const &Sequence, Text_ const &Text)
	{
		for (auto *node : Sequence)
		{
			if (false == save_defined_node(node, Text))
			{
				return false;
			}
		}
		return true;
	}
	template <typename Node_, typename Text_>
	bool save_defined_node(Node_ *Node, Text_ const &Text)
	{
		if (Node == nullptr || Node->from_template())
		{
			return true;
		}
		auto const nodelayer{layers.resolve(Node->layer())};
		if (false == is_output(nodelayer))
		{
			return true;
		}
		auto const lookup{layers.m_sources.find(Node)};
		if (lookup == layers.m_sources.end())
		{
			if (Node->m_editorremoved || false == Node->dirty())
			{
				return true;
			}
			created[nodelayer].emplace_back(Node, Text(*Node, layers.layer(nodelayer).context_insert().offset));
			saved.push_back(Node);
			return true;
		}
		auto const &source{lookup->second};
		if ((false == Node->m_editorremoved && false == Node->dirty()) || false == layers.writable(source.layer))
		{
			return true;
		}
		if (false == load(source.layer))
		{
			return false;
		}
		if (Node->m_editorremoved)
		{
			dropped.push_back(Node);
			push_edit(source.layer, source.span, {}, nullptr);
		}
		else
		{
			push_edit(source.layer, source.span, Text(*Node, source.context.offset), Node);
			saved.push_back(Node);
		}
		rewritten.emplace(Node);
		return true;
	}
	// rewrites the definition of a node the editor moved along with the track it belongs to, with Patch
	template <typename Patch_>
	bool patch_node(basic_node *Node, Patch_ const &Patch)
	{
		auto const lookup{layers.m_sources.find(Node)};
		if (lookup == layers.m_sources.end() || false == Node->dirty())
		{
			return true;
		}
		auto const &source{lookup->second};
		if (false == is_output(layers.resolve(source.layer)))
		{
			return true;
		}
		if (false == layers.writable(source.layer))
		{
			unwritten.push_back(Node->name());
			return true;
		}
		if (false == load(source.layer))
		{
			return false;
		}
		auto const &filetext{content[source.layer]};
		if (source.span.end > static_cast<std::streamoff>(filetext.size()))
		{
			error = "definition of \"" + Node->name() + "\" is out of bounds of file \"" + layers.layer(source.layer).name + "\"";
			return false;
		}
		auto text{filetext.substr(static_cast<std::size_t>(source.span.begin), static_cast<std::size_t>(source.span.end - source.span.begin))};
		std::string patcherror;
		if (false == Patch(text, source, patcherror))
		{
			error = patcherror + " of \"" + Node->name() + "\" in file \"" + layers.layer(source.layer).name + "\"";
			return false;
		}
		push_edit(source.layer, source.span, std::move(text), Node);
		rewritten.emplace(Node);
		saved.push_back(Node);
		return true;
	}
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

// loads text of specified layer file, if it wasn't loaded yet
bool node_layers::save_state::load(layer_handle const Layer)
{
	if (layers.layer(Layer).created || content.count(Layer) != 0)
	{
		return true;
	}
	std::error_code fault;
	auto const filepath{std::filesystem::path(layers.path(Layer))};
	auto const filesize{std::filesystem::file_size(filepath, fault)};
	auto const filetime{fault ? std::filesystem::file_time_type{} : std::filesystem::last_write_time(filepath, fault)};
	std::string filetext;
	if (fault || false == read_file(layers.path(Layer), filetext))
	{
		error = "can't read file \"" + layers.layer(Layer).name + "\"";
		return false;
	}
	if (filesize != layers.layer(Layer).filesize || filetime != layers.layer(Layer).filetime || filetext.size() != filesize)
	{
		// locations of the definitions are no longer valid
		error = "file \"" + layers.layer(Layer).name + "\" was changed outside of the editor since the scenery was loaded";
		return false;
	}
	content.emplace(Layer, std::move(filetext));
	return true;
}

// a layer is saved if it's still a part of the scenery, as a file of its own
bool node_layers::save_state::is_output(layer_handle const Layer) const
{
	return layers.listed(Layer) && false == layers.layer(Layer).removed && false == layers.layer(Layer).binary;
}

void node_layers::save_state::push_edit(layer_handle const Layer, source_span const &Span, std::string Text, basic_node const *Node)
{
	file_patch::edit edit;
	edit.begin = Span.begin;
	edit.end = Span.end;
	edit.length = Text.size();
	edit.text = std::move(Text);
	edit.node = Node;
	patches[Layer].edits.emplace_back(std::move(edit));
}

// changed, created and deleted nodes, the editor data in the comment lines, the includes of templates and the layer changes
bool node_layers::save_state::collect_changes()
{
	if (false == (process_nodes() && relocate_groups() && patch_attached() && save_paths() && save_defined(simulation::Roads.sequence(), &road_text) &&
	              save_defined(simulation::Sweeps.sequence(), &sweep_text) && save_defined(simulation::Junctions.sequence(), &junction_text) &&
	              save_defined(simulation::Roadpoints.sequence(), &roadpoint_text)))
	{
		return false;
	}
	for (auto const &erased : layers.m_erased)
	{
		push_edit(erased.first, erased.second, {}, nullptr);
	}
	mark_lines();
	return place_templates() && change_layers();
}

// changed and deleted model instances and memory cells
bool node_layers::save_state::process_nodes()
{
	for (auto const *instance : simulation::Instances.sequence())
	{
		if (instance != nullptr && false == process(instance, instance))
		{
			return false;
		}
	}
	for (auto const *memorycell : simulation::Memory.sequence())
	{
		if (memorycell != nullptr && false == process(memorycell, nullptr))
		{
			return false;
		}
	}
	return true;
}

bool node_layers::save_state::process(basic_node const *Node, TAnimModel const *Model)
{
	auto const nodelayer{layers.resolve(Node->layer())};
	if (false == is_output(nodelayer))
	{
		return true;
	}
	if (Model != nullptr && Groups.relocating(Node))
	{
		// written anew along with its group, below
		return true;
	}
	if (auto const lookup{layers.m_sources.find(Node)}; lookup != layers.m_sources.end())
	{
		// defined in a scenery file. rewrite the placement in its definition if it's no longer the one it was loaded with
		return rewrite_placement(Node, Model, lookup->second);
	}
	if (Model != nullptr && false == Node->from_template())
	{
		// created in the editor. the definition is added to the layer file, with the placement in effect at the place it goes to
		std::string text;
		if (false == placed_definition(Model, nodelayer, text))
		{
			return false;
		}
		created[nodelayer].emplace_back(Node, std::move(text));
	}
	return true;
}

placement_change node_layers::save_state::change_of(basic_node const *Node, TAnimModel const *Model, node_source const &Source) const
{
	placement_change change;
	change.location = Node->location();
	change.angles = (Model != nullptr ? Model->Angles() : glm::vec3{0.f});
	change.scale = (Model != nullptr ? Model->Scale() : glm::vec3{1.f});
	change.moved = glm::distance(change.location, Source.location) > 1e-4;
	change.rotated = false == glm::all(glm::epsilonEqual(change.angles, Source.angles, 1e-3f));
	change.scaled = false == glm::all(glm::epsilonEqual(change.scale, Source.scale, 1e-4f));
	change.renamed = Model != nullptr && layers.m_renamed.find(Node) != layers.m_renamed.end();
	auto const *cell{Model == nullptr ? static_cast<TMemCell const *>(Node) : nullptr};
	if (cell != nullptr && cell->Track != nullptr)
	{
		change.track = cell->Track->name();
	}
	return change;
}

bool node_layers::save_state::rewrite_placement(basic_node const *Node, TAnimModel const *Model, node_source const &Source)
{
	if (false == layers.writable(Source.layer))
	{
		// the editor doesn't let these be changed, as one definition stands for more than one node
		return true;
	}
	auto const change{change_of(Node, Model, Source)};
	if (false == (change.moved || change.rotated || change.scaled || change.renamed || (false == change.track.empty() && Node->dirty()))) // NOSONAR
	{
		return true;
	}
	if (false == load(Source.layer))
	{
		return false;
	}
	auto const &filetext{content[Source.layer]};
	if (Source.span.end > static_cast<std::streamoff>(filetext.size()))
	{
		error = "definition of \"" + Node->name() + "\" is out of bounds of file \"" + layers.layer(Source.layer).name + "\""; // NOSONAR
		return false;
	}
	auto text{filetext.substr(static_cast<std::size_t>(Source.span.begin), static_cast<std::size_t>(Source.span.end - Source.span.begin))};
	std::string patcherror;
	if (false == patch_placement(text, Node->name(), Model != nullptr, Source, change, patcherror))
	{
		error = patcherror + " of \"" + Node->name() + "\" in file \"" + layers.layer(Source.layer).name + "\""; // NOSONAR
		return false;
	}
	if (text.size() == static_cast<std::size_t>(Source.span.end - Source.span.begin) && filetext.compare(static_cast<std::size_t>(Source.span.begin), text.size(), text) == 0)
	{
		return true;
	}
	push_edit(Source.layer, Source.span, std::move(text), Node);
	rewritten.emplace(Node);
	return true;
}

// the definition of a model instance written anew in the file of specified layer, with the placement in effect at the place it goes to
bool node_layers::save_state::placed_definition(TAnimModel const *Model, layer_handle const Target, std::string &Text)
{
	Model->export_as_text(Text);
	Text.erase(Text.find_last_not_of(" \t\r\n") + 1);
	auto const &context{layers.layer(Target).context_insert()};
	if (context.matches(layer_context()))
	{
		return true;
	}
	model_placement placement;
	placement.rotation = context.rotation;
	placement.location = context.to_local(Model->location());
	placement.angles = Model->Angles();
	placement.scale = Model->Scale() / context.scale;
	std::string patcherror;
	if (patch_model(Text, placement, patcherror))
	{
		return true;
	}
	error = patcherror + " of \"" + Model->name() + "\"";
	return false;
}

// models grouped or ungrouped in the editor: their definitions are taken out of where they are and written anew, a group
// as one block between group and endgroup in the layer of its first model, a model of a dissolved group on its own
bool node_layers::save_state::relocate_groups()
{
	for (auto const *node : Groups.loose())
	{
		auto const *model{dynamic_cast<TAnimModel const *>(node)};
		if (model == nullptr || model->from_template() || false == is_output(layers.resolve(node->layer())))
		{
			continue;
		}
		if (false == relocate(model, layers.resolve(node->layer()), created[layers.resolve(node->layer())]))
		{
			return false;
		}
	}
	for (auto const group : Groups.rewritten())
	{
		if (false == relocate_group(group))
		{
			return false;
		}
	}
	return true;
}

bool node_layers::save_state::relocate_group(group_handle const Group)
{
	auto const &members{Groups.group(Group).nodes};
	if (members.empty() || false == is_output(layers.resolve(members.front()->layer())))
	{
		return true;
	}
	auto const target{layers.resolve(members.front()->layer())};
	auto &output{created[target]};
	// NOTE: lines with no node are written as they are, without a place in the bookkeeping
	output.emplace_back(nullptr, "group");
	for (auto const *node : members)
	{
		auto const *model{dynamic_cast<TAnimModel const *>(node)};
		if (model == nullptr || model->from_template())
		{
			continue;
		}
		if (false == relocate(model, target, output))
		{
			return false;
		}
	}
	output.emplace_back(nullptr, "endgroup");
	return true;
}

bool node_layers::save_state::relocate(TAnimModel const *Model, layer_handle const Target, definitions &Output)
{
	if (auto const lookup{layers.m_sources.find(Model)}; lookup != layers.m_sources.end())
	{
		auto const &source{lookup->second};
		if (false == layers.writable(source.layer))
		{
			// one definition stands for more than one node, it stays where it is
			return true;
		}
		if (false == load(source.layer))
		{
			return false;
		}
		push_edit(source.layer, source.span, {}, nullptr);
		rewritten.emplace(Model);
	}
	std::string text;
	if (false == placed_definition(Model, Target, text))
	{
		return false;
	}
	Output.emplace_back(Model, std::move(text));
	relocated.emplace(Model);
	return true;
}

// traction and event launchers are rewritten when the editor moved them along with the track they belong to
bool node_layers::save_state::patch_attached()
{
	for (auto *traction : simulation::Traction.sequence())
	{
		if (traction != nullptr &&
		    false == patch_node(traction, [traction](std::string &Text, node_source const &Source, std::string &Error) { return patch_traction(Text, *traction, Source.context.offset, Error); }))
		{
			return false;
		}
	}
	for (auto *launcher : simulation::Events.launchers())
	{
		if (launcher != nullptr &&
		    false == patch_node(launcher, [launcher](std::string &Text, node_source const &Source, std::string &Error) { return patch_launcher(Text, Source.context.to_local(launcher->location()), Error); }))
		{
			return false;
		}
	}
	return true;
}

bool node_layers::save_state::save_paths()
{
	for (auto *path : simulation::Paths.sequence())
	{
		if (false == save_path(path))
		{
			return false;
		}
	}
	return true;
}

bool node_layers::save_state::save_path(TTrack *Path)
{
	if (Path == nullptr || (Path->iCategoryFlag & 0x80) != 0 || Path->m_road != nullptr || Path->from_template())
	{
		return true;
	}
	auto const nodelayer{layers.resolve(Path->layer())};
	if (false == is_output(nodelayer))
	{
		return true;
	}
	auto const lookup{layers.m_sources.find(Path)};
	if (lookup == layers.m_sources.end())
	{
		if (Path->m_editorremoved || false == Path->dirty())
		{
			return true;
		}
		// a file included more than once would get a copy of the new path at every inclusion
		auto const target{layers.writable(nodelayer) ? nodelayer : root};
		created[target].emplace_back(Path, path_text(*Path, layers.layer(target).context_insert().offset));
		saved.push_back(Path);
		return true;
	}
	auto const &source{lookup->second};
	if (false == Path->m_editorremoved && false == Path->dirty())
	{
		return true;
	}
	if (false == layers.writable(source.layer))
	{
		unwritten.push_back(Path->name().empty() ? std::string{STR_C("(no name)")} : Path->name());
		return true;
	}
	if (false == load(source.layer))
	{
		return false;
	}
	if (Path->m_editorremoved)
	{
		push_edit(source.layer, source.span, {}, nullptr);
		dropped.push_back(Path);
	}
	else
	{
		push_edit(source.layer, source.span, path_text(*Path, source.context.offset), Path);
		saved.push_back(Path);
	}
	rewritten.emplace(Path);
	return true;
}

// editor data in the comment lines: the old lines go, the new ones are added to the file like new definitions
void node_layers::save_state::mark_lines()
{
	for (auto const &marking : layers.m_marking)
	{
		replace_marked(marking.first.first, marking.first.second, marking.second);
	}
	// the lines read from the letters the marks had before go to the mark they have now, also those the editor didn't change
	for (auto const &[markedlayer, lines] : layers.m_marked)
	{
		move_marked(markedlayer, lines);
	}
}

void node_layers::save_state::replace_marked(layer_handle const Markedlayer, std::string const &Mark, std::vector<std::string> const &Lines)
{
	if (auto const lookup{layers.m_marked.find(Markedlayer)}; lookup != layers.m_marked.end())
	{
		for (auto const &line : lookup->second)
		{
			if (line.mark == Mark)
			{
				push_edit(Markedlayer, line.span, {}, nullptr);
			}
		}
	}
	auto const target{layers.resolve(Markedlayer)};
	if (Lines.empty() || false == is_output(target))
	{
		return;
	}
	for (auto const &text : Lines)
	{
		markedtext[target].push_back(Mark + ' ' + text);
	}
}

void node_layers::save_state::move_marked(layer_handle const Markedlayer, std::vector<marked_line> const &Lines)
{
	auto const target{layers.resolve(Markedlayer)};
	if (layers.layer(Markedlayer).dead || false == is_output(target) || false == layers.writable(Markedlayer))
	{
		return;
	}
	for (auto const &line : Lines)
	{
		if (line.moved && false == layers.m_marking.contains({Markedlayer, line.mark}))
		{
			push_edit(Markedlayer, line.span, {}, nullptr);
			markedtext[target].push_back(line.mark + ' ' + line.text);
		}
	}
}

// includes of templates
bool node_layers::save_state::place_templates()
{
	for (std::size_t idx = 0; idx < layers.m_instances.size(); ++idx)
	{
		if (false == place_template(idx))
		{
			return false;
		}
	}
	return true;
}

bool node_layers::save_state::place_template(std::size_t const Index)
{
	auto const &included{layers.m_instances[Index]};
	if (included.dead)
	{
		return true;
	}
	if (included.span.valid())
	{
		if (included.removed)
		{
			push_edit(included.layer, included.span, {}, nullptr);
		}
		else if (false == included.directive.empty())
		{
			// changed in the editor, the new directive takes the place of the one in the file
			file_patch::edit edit;
			edit.begin = included.span.begin;
			edit.end = included.span.end;
			edit.length = included.directive.size();
			edit.text = included.directive;
			edit.instance = static_cast<instance_handle>(Index + 1);
			patches[included.layer].edits.emplace_back(std::move(edit));
		}
		return true;
	}
	if (included.removed || included.directive.empty())
	{
		return true;
	}
	auto const target{layers.resolve(included.layer)};
	if (false == is_output(target))
	{
		return true;
	}
	if (false == included.context.matches(layers.layer(target).context_insert()))
	{
		// shouldn't happen, merge of layers is refused when it'd lead to this
		error = format(STR_C("include \"%s\" was placed for different origin, rotation or scale than its layer file receives it with"), included.file->c_str());
		return false;
	}
	placed[target].emplace_back(static_cast<instance_handle>(Index + 1));
	return true;
}

// layer changes
bool node_layers::save_state::change_layers()
{
	for (std::size_t idx = 0; idx < layers.m_layers.size(); ++idx)
	{
		if (false == change_layer(static_cast<layer_handle>(idx + 1)))
		{
			return false;
		}
	}
	return true;
}

bool node_layers::save_state::change_layer(layer_handle const Candidate)
{
	auto const &source{layers.layer(Candidate)};
	if (source.dead)
	{
		return true;
	}
	auto const hassite{false == source.created && source.sites.size() == 1 && false == source.sites.front().fixed && source.sites.front().span.valid()};
	if (source.merged != null_handle)
	{
		merge_layer(Candidate, hassite);
	}
	else if (source.removed)
	{
		if (hassite)
		{
			push_edit(source.sites.front().parent, source.sites.front().span, {}, nullptr);
			detached.emplace(Candidate);
		}
	}
	else if (source.created)
	{
		return include_created(Candidate);
	}
	return true;
}

void node_layers::save_state::merge_layer(layer_handle const Candidate, bool const Hassite)
{
	auto const &source{layers.layer(Candidate)};
	auto const target{layers.resolve(Candidate)};
	absorbed.emplace(Candidate);
	if (false == Hassite)
	{
		appended[target].emplace_back(Candidate);
		return;
	}
	file_patch::edit edit;
	edit.begin = source.sites.front().span.begin;
	edit.end = source.sites.front().span.end;
	if (layers.resolve(source.sites.front().parent) == target)
	{
		edit.inlined = Candidate;
	}
	else
	{
		appended[target].emplace_back(Candidate);
	}
	patches[source.sites.front().parent].edits.emplace_back(std::move(edit));
	detached.emplace(Candidate);
}

// include directive for a file created in the editor, on a line of its own
bool node_layers::save_state::include_created(layer_handle const Candidate)
{
	auto const &source{layers.layer(Candidate)};
	auto const parent{source.parent};
	auto const directive{"include " + source.name + " end"};
	file_patch::edit edit;
	edit.site = Candidate;
	edit.length = directive.size();
	auto const anchor{source.anchor};
	if (layers.valid(anchor) && layers.layer(anchor).sites.size() == 1 && layers.layer(anchor).sites.front().parent == parent && layers.layer(anchor).sites.front().span.valid())
	{
		edit.begin = layers.layer(anchor).sites.front().span.end;
		edit.end = edit.begin;
		edit.text = eol + directive;
		edit.lead = eol.size();
	}
	else
	{
		// NOTE: negative location stands for the place the file receives additions at
		edit.begin = -1;
		edit.end = -1;
		if (false == layers.layer(parent).created)
		{
			if (false == load(parent))
			{
				return false;
			}
			// dropped by compose() if the text ahead ends with a line break already
			edit.text = eol;
			edit.lead = eol.size();
		}
		edit.text += directive + eol;
	}
	patches[parent].edits.emplace_back(std::move(edit));
	return true;
}

// files to write
void node_layers::save_state::list_outputs(std::vector<std::string> const &Rootstatements, std::vector<std::string> const &Trailingstatements,
                                           std::map<layer_handle, std::vector<std::string>> const &Statements)
{
	for (std::size_t idx = 0; idx < layers.m_layers.size(); ++idx)
	{
		auto const candidate{static_cast<layer_handle>(idx + 1)};
		if (false == is_output(candidate))
		{
			continue;
		}
		if (layers.layer(candidate).created || patches.count(candidate) != 0 || appended.count(candidate) != 0 || created.count(candidate) != 0 || placed.count(candidate) != 0 ||
		    markedtext.contains(candidate) || Statements.contains(candidate) || (candidate == root && (false == Rootstatements.empty() || false == Trailingstatements.empty())))
		{
			outputs.emplace_back(candidate);
		}
	}
}

// text of the merged layers travels with the file they end up in, but changes made to it are still theirs
bool node_layers::save_state::load_composed()
{
	composed = outputs;
	composed.insert(std::end(composed), std::begin(absorbed), std::end(absorbed));
	for (auto const candidate : composed)
	{
		if (false == load(candidate))
		{
			return false;
		}
	}
	return true;
}

// what is defined in the loaded files and isn't rewritten moves along with the text
void node_layers::save_state::track_unchanged()
{
	for (auto const &source : layers.m_sources)
	{
		if (content.count(source.second.layer) != 0 && rewritten.count(source.first) == 0)
		{
			nodes[source.second.layer].emplace_back(source.first);
		}
	}
	for (std::size_t idx = 0; idx < layers.m_layers.size(); ++idx)
	{
		auto const candidate{static_cast<layer_handle>(idx + 1)};
		auto const &included{layers.layer(candidate)};
		if (included.dead || included.created || detached.count(candidate) != 0 || included.sites.size() != 1 || included.sites.front().fixed || false == included.sites.front().span.valid())
		{
			continue;
		}
		includes[included.sites.front().parent].emplace_back(candidate);
	}
	for (std::size_t idx = 0; idx < layers.m_instances.size(); ++idx)
	{
		auto const &included{layers.m_instances[idx]};
		if (false == included.dead && false == included.removed && included.directive.empty() && included.span.valid() && content.count(included.layer) != 0)
		{
			instances[included.layer].emplace_back(static_cast<instance_handle>(idx + 1));
		}
	}
}

bool node_layers::save_state::compose_outputs(std::vector<std::string> const &Rootstatements, std::vector<std::string> const &Trailingstatements,
                                              std::map<layer_handle, std::vector<std::string>> const &Statements)
{
	for (auto const output : outputs)
	{
		if (false == compose_output(output, Rootstatements, Trailingstatements, Statements))
		{
			return false;
		}
	}
	return true;
}

bool node_layers::save_state::compose_output(layer_handle const Output, std::vector<std::string> const &Rootstatements, std::vector<std::string> const &Trailingstatements,
                                             std::map<layer_handle, std::vector<std::string>> const &Statements)
{
	auto &text{texts[Output]};
	auto const fileeol{detect_eol(content.count(Output) != 0 ? content[Output] : std::string{}, eol)};
	if (layers.layer(Output).created)
	{
		// NOTE: the parser treats an empty file as a missing one
		text.text = (layers.layer(Output).header.empty() ? std::string{"// scenery layer created in the editor"} : layers.layer(Output).header) + fileeol;
	}
	composition body;
	if (false == layers.compose(*this, Output, body, 0))
	{
		return false;
	}
	text.append(body);
	// what the editor adds to the file
	composition added;
	if (false == compose_merged(Output, fileeol, added))
	{
		return false;
	}
	add_definitions(Output, fileeol, added);
	if (auto const lookup{markedtext.find(Output)}; lookup != markedtext.end())
	{
		append_lines(added.text, lookup->second, fileeol);
	}
	if (Output == root)
	{
		append_lines(added.text, Rootstatements, fileeol);
	}
	if (auto const lookup{Statements.find(Output)}; lookup != Statements.end())
	{
		append_lines(added.text, lookup->second, fileeol);
	}
	if (Output == root && false == Trailingstatements.empty())
	{
		composition trailing;
		for (auto const &statement : Trailingstatements)
			trailing.text += statement + fileeol;
		ensure_newline(text.text, fileeol);
		text.append(trailing);
	}
	if (false == added.text.empty())
	{
		text.add(added, fileeol);
	}
	return true;
}

// the merged layers placed at the end of the file
bool node_layers::save_state::compose_merged(layer_handle const Output, std::string const &Eol, composition &Added)
{
	auto const lookup{appended.find(Output)};
	if (lookup == appended.end())
	{
		return true;
	}
	for (auto const merged : lookup->second)
	{
		composition mergedtext;
		if (false == layers.compose(*this, merged, mergedtext, 0))
		{
			return false;
		}
		ensure_newline(Added.text, Eol);
		Added.append(mergedtext);
	}
	return true;
}

// the definitions created in the editor and the include directives placed in it
void node_layers::save_state::add_definitions(layer_handle const Output, std::string const &Eol, composition &Added) const
{
	if (auto const lookup{created.find(Output)}; lookup != created.end())
	{
		for (auto const &definition : lookup->second)
		{
			ensure_newline(Added.text, Eol);
			auto const offset{static_cast<std::streamoff>(Added.text.size())};
			if (definition.first != nullptr)
			{
				Added.nodes.push_back({definition.first, {offset, offset + static_cast<std::streamoff>(definition.second.size())}, true});
			}
			Added.text += definition.second + Eol;
		}
	}
	if (auto const lookup{placed.find(Output)}; lookup != placed.end())
	{
		for (auto const included : lookup->second)
		{
			auto const &directive{layers.instance(included).directive};
			ensure_newline(Added.text, Eol);
			auto const offset{static_cast<std::streamoff>(Added.text.size())};
			Added.instances.push_back({included, {offset, offset + static_cast<std::streamoff>(directive.size())}});
			Added.text += directive + Eol;
		}
	}
}

// new content goes to temporary files first, so a failure half way through doesn't leave the scenery with only a part of the changes
bool node_layers::save_state::write_temporary()
{
	for (auto const output : outputs)
	{
		auto const lookup{content.find(output)};
		if (lookup != content.end() && lookup->second == texts[output].text)
		{
			continue;
		}
		if (false == write_temporary(output))
		{
			return false;
		}
		written.emplace_back(output);
	}
	return true;
}

bool node_layers::save_state::write_temporary(layer_handle const Output)
{
	std::error_code fault;
	auto const filepath{std::filesystem::path(layers.path(Output))};
	if (filepath.has_parent_path())
	{
		std::filesystem::create_directories(filepath.parent_path(), fault);
	}
	std::ofstream file{layers.path(Output) + ".tmp", std::ios_base::binary | std::ios_base::trunc};
	file << texts[Output].text;
	file.close();
	if (false == file.fail())
	{
		return true;
	}
	drop_temporary();
	std::filesystem::remove(layers.path(Output) + ".tmp", fault);
	error = format(STR_C("can't write file \"%s\""), layers.layer(Output).name.c_str());
	return false;
}

// the temporary files written so far go
void node_layers::save_state::drop_temporary() const
{
	std::error_code fault;
	for (auto const leftover : written)
	{
		std::filesystem::remove(layers.path(leftover) + ".tmp", fault);
	}
}

// a file which loses its ground to the terrain of the editor keeps a copy of itself each time, whatever was saved before.
// without the copy nothing is replaced
bool node_layers::save_state::back_up_ground(std::vector<std::string> &Backups)
{
	auto const now{std::time(nullptr)};
	std::array<char, 32> stamp{};
	std::strftime(stamp.data(), stamp.size(), "%Y%m%d-%H%M%S", std::localtime(&now));
	for (auto const output : written)
	{
		if (false == back_up(output, stamp.data(), Backups))
		{
			return false;
		}
	}
	return true;
}

bool node_layers::save_state::back_up(layer_handle const Output, char const *Stamp, std::vector<std::string> &Backups)
{
	auto const filepath{layers.path(Output)};
	bool const losesground{std::ranges::any_of(layers.m_shapes, [Output](shape_source const &Shape) { return Shape.erased && Shape.layer == Output; })};
	std::error_code fault;
	if (false == losesground || false == std::filesystem::exists(filepath, fault))
	{
		return true;
	}
	auto const backup{filepath + ".bak-" + Stamp + "-przed-terenem-edytora"};
	std::error_code copyerror;
	if (false == std::filesystem::copy_file(filepath, backup, std::filesystem::copy_options::overwrite_existing, copyerror))
	{
		drop_temporary();
		error = format(STR_C("can't make a backup of file \"%s\": %s"), layers.layer(Output).name.c_str(), copyerror.message().c_str());
		return false;
	}
	WriteLog("Scenery save: the triangles of the ground go from \"" + layers.layer(Output).name + "\", its previous content is kept in \"" + backup + "\"");
	Backups.emplace_back(backup);
	return true;
}

// the temporary files take the place of the scenery files
bool node_layers::save_state::replace_files(save_result &Result)
{
	std::vector<layer_handle> replaced;
	for (auto const output : written)
	{
		if (false == replace_file(output, replaced))
		{
			Result.files.clear();
			return false;
		}
		replaced.emplace_back(output);
		Result.files.emplace_back(layers.layer(output).name);
	}
	return true;
}

bool node_layers::save_state::replace_file(layer_handle const Output, std::vector<layer_handle> const &Replaced)
{
	auto const filepath{layers.path(Output)};
	std::error_code fault;
	// the first save of a file leaves its original content next to it
	if (false == layers.layer(Output).editormade && std::filesystem::exists(filepath, fault) && false == std::filesystem::exists(filepath + ".bak", fault))
	{
		std::filesystem::copy_file(filepath, filepath + ".bak", fault);
	}
	std::filesystem::rename(filepath + ".tmp", filepath, fault);
	if (false == static_cast<bool>(fault))
	{
		return true;
	}
	// e.g. the file is held open by another program. take back what was done so far: the scenery files
	// have to stay in step with each other, and with what we know about them
	auto const cause{fault.message()};
	restore(Replaced);
	drop_temporary();
	error = format(STR_C("can't replace file \"%s\": %s"), layers.layer(Output).name.c_str(), cause.c_str());
	return false;
}

void node_layers::save_state::restore(std::vector<layer_handle> const &Replaced) const
{
	std::error_code fault;
	for (auto const restored : Replaced)
	{
		auto const lookup{content.find(restored)};
		if (lookup != content.end())
		{
			std::ofstream{layers.path(restored), std::ios_base::binary | std::ios_base::trunc} << lookup->second;
			layers.stat(restored);
		}
		else
		{
			std::filesystem::remove(layers.path(restored), fault);
		}
	}
}

// the scenery files now match the scene, update the bookkeeping to reflect it
void node_layers::save_state::update_bookkeeping()
{
	Groups.saved();
	for (auto const output : outputs)
	{
		update_nodes(output);
		update_includes(output);
	}
	auto removedterrain{retire_layers()};
	forget_changes();
	removedterrain |= layers.m_shapeserased;
	layers.m_shapeserased = false;
	drop_terrain_cache(removedterrain);
}

void node_layers::save_state::update_nodes(layer_handle const Output)
{
	for (auto const &place : texts[Output].nodes)
	{
		auto &source{layers.m_sources[place.node]};
		auto const isnew{source.layer == null_handle};
		source.layer = Output;
		source.span = place.span;
		// a definition written anew elsewhere is in the placement of the place it went to, as a new one is
		if (isnew || relocated.count(place.node) != 0)
		{
			source.context = layers.layer(Output).context_insert();
		}
		if (isnew || place.rewritten)
		{
			auto const *model{dynamic_cast<TAnimModel const *>(place.node)};
			source.location = place.node->location();
			source.angles = (model != nullptr ? model->Angles() : glm::vec3{0.f});
			source.scale = (model != nullptr ? model->Scale() : glm::vec3{1.f});
		}
	}
}

void node_layers::save_state::update_includes(layer_handle const Output)
{
	for (auto const &place : texts[Output].instances)
	{
		auto &included{layers.m_instances[place.instance - 1]};
		included.layer = Output;
		included.span = place.span;
		included.directive.clear();
	}
	for (auto const &place : texts[Output].sites)
	{
		auto &included{layers.layer(place.layer)};
		if (included.sites.empty())
		{
			// directive of a layer created in the editor
			included.sites.emplace_back();
		}
		included.sites.front().parent = Output;
		included.sites.front().span = place.span;
		included.parent = Output;
	}
	// the mark of the scenario initialization moved along with the text around it, or came with the text of a merged layer
	auto &savedlayer{layers.layer(Output)};
	if (false == savedlayer.init.valid() && texts[Output].init.valid())
	{
		for (std::size_t idx = 0; idx < layers.m_layers.size(); ++idx)
		{
			if (layers.m_layers[idx].init.valid() && layers.m_layers[idx].merged != null_handle && layers.resolve(static_cast<layer_handle>(idx + 1)) == Output)
			{
				savedlayer.context_init = layers.m_layers[idx].context_init;
			}
		}
	}
	savedlayer.init = texts[Output].init;
}

// the merged and removed layers are no longer a part of the scenery. returns: true if a removed one had ground in it
bool node_layers::save_state::retire_layers()
{
	auto removedterrain{false};
	for (std::size_t idx = 0; idx < layers.m_layers.size(); ++idx)
	{
		auto &retired{layers.layer(static_cast<layer_handle>(idx + 1))};
		if (retired.dead)
		{
			continue;
		}
		if (retired.merged != null_handle || retired.removed)
		{
			removedterrain |= retired.removed && (retired.items[static_cast<std::size_t>(layer_item::shape)] + retired.items[static_cast<std::size_t>(layer_item::lines)] > 0);
			retired.dead = true;
			retired.sites.clear();
		}
		retired.created = false;
		retired.anchor = null_handle;
	}
	return removedterrain;
}

// the changes the files received are no longer pending
void node_layers::save_state::forget_changes()
{
	for (auto const output : written)
	{
		layers.stat(output);
		layers.scan(output);
	}
	// the lines of the merged layers are now in the files of their targets, read again with them: all of them, the ones the
	// editor didn't change too, or the next save would write the lines moved off the letters of the starters twice
	for (auto marked{layers.m_marked.begin()}; marked != layers.m_marked.end();)
	{
		marked = (layers.resolve(marked->first) != marked->first ? layers.m_marked.erase(marked) : std::next(marked));
	}
	layers.m_marking.clear();
	for (auto &included : layers.m_instances)
	{
		// removed includes are gone from the files now
		included.dead = included.dead || included.removed;
	}
	layers.m_erased.clear();
	layers.m_renamed.clear();
	// the shapes of the rewritten files are somewhere else in them now
	std::erase_if(layers.m_shapes, [this](shape_source const &Shape) { return std::ranges::find(composed, Shape.layer) != std::end(composed); });
	for (auto *node : saved)
	{
		node->m_dirty = false;
	}
	for (auto const *node : dropped)
	{
		layers.m_sources.erase(node);
	}
}

// the binary terrain file still holds geometry of a dropped layer, have it rebuilt on the next load
void node_layers::save_state::drop_terrain_cache(bool const Removedterrain)
{
	if (false == Removedterrain)
	{
		return;
	}
	auto terrainfile{layers.layer(root).name};
	while (false == terrainfile.empty() && terrainfile.front() == '$')
	{
		terrainfile.erase(0, 1);
	}
	erase_extension(terrainfile);
	if (std::error_code fault; std::filesystem::remove(Global.asCurrentSceneryPath + terrainfile + ".sbt", fault))
	{
		WriteLog("Scenery save: binary terrain file removed, it'll be generated again on the next load");
	}
}

// writes changes made in the editor to the scenery files
save_result node_layers::save(std::vector<std::string> const &Rootstatements, std::vector<std::string> const &Trailingstatements, std::map<layer_handle, std::vector<std::string>> const &Statements)
{
	save_result result;
	if (m_layers.empty())
	{
		result.message = STR_C("The scenery wasn't opened for editing, there are no layers to save");
		return result;
	}
	save_state state{*this};
	auto const fail = [&result](std::string const &Message) {
		result.message = format(STR_C("Scenery not saved: %s"), Message.c_str());
		ErrorLog(result.message);
		return result;
	};
	if (false == state.load(state.root))
	{
		return fail(state.error);
	}
	state.eol = detect_eol(state.content[state.root], "\r\n");
	if (false == state.collect_changes())
	{
		return fail(state.error);
	}
	state.list_outputs(Rootstatements, Trailingstatements, Statements);
	if (false == state.load_composed())
	{
		return fail(state.error);
	}
	state.track_unchanged();
	if (false == state.compose_outputs(Rootstatements, Trailingstatements, Statements))
	{
		return fail(state.error);
	}
	// with everything worked out it's safe to touch the files
	if (false == state.write_temporary() || false == state.back_up_ground(result.backups) || false == state.replace_files(result))
	{
		return fail(state.error);
	}
	for (auto const &file : result.files)
	{
		WriteLog("Scenery save: file \"" + file + "\" updated");
	}
	state.update_bookkeeping();

	result.success = true;
	result.message = saved_message(result);
	if (false == state.unwritten.empty())
	{
		auto const names{unwritten_names(state.unwritten)};
		WriteLog(std::format("Scenery save: {} changed nodes not saved, their files are included with parameters or more than once: {}", state.unwritten.size(), names), logtype::generic);
		auto const warning{format(STR_C("Changed nodes left unsaved (their files are included with parameters or more than once): %zu, %s"), state.unwritten.size(), names.c_str())};
		result.message = result.files.empty() ? warning : result.message + ". " + warning;
	}
	return result;
}

} // namespace scene

//---------------------------------------------------------------------------
