/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace scene
{

// model file and replacable skin of a model instance, in the form TAnimModel::Load() gives the tokens which name them
struct model_appearance
{
	std::string model; // lower case, forward slashes
	std::string texture; // forward slashes
};

// definition of a model instance read from scenery text:
// node <max range> <min range> <name> model <x> <y> <z> <angle> <model> <texture> [angles <x> <y> <z>] [scale <x> <y> <z>] [notransition] endmodel
struct model_entry
{
	double range_max{0.0};
	double range_min{0.0};
	std::string_view name; // the way the text spells it. NOTE: refers to the text
	glm::dvec3 location{0.0};
	float angle{0.f}; // rotation around the vertical axis
	glm::vec3 angles{0.f}; // content of the angles block, if there's one
	glm::vec3 scale{1.f}; // content of the scale block, if there's one
	bool has_angles{false};
	bool has_scale{false};
	bool notransition{false};
	std::uint32_t appearance{0}; // model and texture of the instance, index into the appearances of the block holding the entry
	std::size_t begin{0}; // location of the definition in the text: from the start of its node keyword...
	std::size_t end{0}; // ...to the end of its closing keyword
	std::size_t line{0}; // line breaks in the text ahead of the definition, counted the way the parser counts them for a node
};

// definitions read from one piece of the text
struct model_entry_block
{
	std::vector<model_entry> entries;
	std::vector<model_appearance> appearances;
};

// run of definitions a text starts with
struct model_entry_run
{
	std::vector<model_entry_block> blocks; // in the order of the text; no block is empty
	std::size_t length{0}; // amount of the text taken by the definitions
	std::size_t linebreaks{0}; // number of line breaks in that part of the text
};

// how the text is divided between the threads
struct model_reader_setup
{
	std::size_t lead{64 * 1024}; // amount of the text read before the threads get involved; most runs of definitions are short
	std::size_t window{8 * 1024 * 1024}; // most of the text a single call goes through past the lead
	std::size_t share{256 * 1024}; // least amount of the text worth a thread of its own
	unsigned int threads{0}; // most threads put to work; 0: as many as the hardware runs at a time
};

// reads the definitions of model instances provided text starts with. the reading ends where the text holds something else
// than a definition the reader knows inside out: another kind of node or directive, a block comment, a definition of
// terrain, one with lights or anything else not listed above, with quoted text, an include or a comment glued to a token.
// that part is left for the scenery parser, so the outcome is the same as if the parser went through all of the text.
// Keywordread: the node keyword of the first definition was already taken off the text; begin and line of that entry aren't set
model_entry_run read_model_entries(std::string_view Text, bool Keywordread, model_reader_setup const &Setup = model_reader_setup());

} // namespace scene

//---------------------------------------------------------------------------
