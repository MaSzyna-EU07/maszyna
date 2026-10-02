/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <vector>

// description of an *.inc scenery template: what the template is, and what its (pN) parameters mean.
// it's kept in the template file itself, as a YAML document made of the comment lines starting with "//$e", e.g.
//   //$e version: 1
//   //$e name: Semaphore, 5 lights
//   //$e params:
//   //$e   - {id: 1, role: name, label: Name}
// being a comment it's invisible to the scenery parser and to the older tools
struct include_parameter
{
	int id{0}; // number of the (pN) placeholder
	std::string role{"free"}; // what the value is; one of editor_includes::roles
	std::string label; // name to show to the user
	std::string value; // default value
};

struct include_info
{
	std::string name;
	std::string category;
	std::string description;
	std::vector<include_parameter> parameters;

	// description of specified parameter, created if there's none
	include_parameter &parameter(int Id);
};

namespace editor_includes
{

// recognized parameter roles. "free" is a value typed in by the user, with no meaning to the editor;
// the placement roles (pos.*, rot.*) and "name" are the ones the editor can fill in on its own
extern std::vector<std::string> const roles;

// reads description of specified template, located in the scenery directory. a template without the description
// yields an empty one. returns: false if the file can't be read or its description isn't a valid YAML document
bool load(std::string const &File, include_info &Info, std::string &Error);
// stores provided description in specified template. entries of the existing description which aren't
// a part of include_info are preserved. returns: false if the file can't be updated
bool save(std::string const &File, include_info const &Info, std::string &Error);
// highest number of (pN) placeholder used by specified template, 0 if it has none
int parameter_count(std::string const &File);
// sets roles of the parameters whose use in the template makes their meaning clear. roles already set are left alone
void suggest(std::string const &File, include_info &Info);

} // namespace editor_includes
