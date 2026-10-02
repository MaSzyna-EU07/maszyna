/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <memory>
#include <optional>
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
// true if value of a parameter with specified role is supplied by the editor when the template is placed
bool automatic(std::string const &Role);
// true if the description has everything it takes to place the template straight from the node bank: a name, and
// either a default value or a role filled in by the editor for each of specified number of parameters.
// optionally tells what is missing
bool complete(include_info const &Info, int Parameters, std::string *Issue = nullptr);
// builds include directive which places specified template. Location and Yaw are the placement to pass through
// the parameters with matching roles, in the coordinates of the place the directive goes to; without Yaw the
// rotation parameters receive their default values
std::string directive(std::string const &File, include_info const &Info, int Parameters, glm::dvec3 const &Location, std::optional<float> Yaw);

// splits include directive into the name of the included file and the values of the parameters, both as they're
// written. returns: false if the text isn't a complete include directive
bool parse_directive(std::string const &Directive, std::string &File, std::vector<std::string> &Values);
// builds include directive out of the name of the included file and the values of the parameters
std::string compose_directive(std::string const &File, std::vector<std::string> const &Values);
// text of a number passed as a parameter value
std::string number(double Value);
// number of the parameter with specified role, 0 if the description has none
int parameter_with_role(include_info const &Info, std::string const &Role);

// marks node bank template which stands for a scenery template rather than for definition of a single node
extern std::string const directive_mark;

} // namespace editor_includes

// *.inc template found in the scenery directory
struct include_entry
{
	std::string file; // name of the template, relative to the scenery directory
	std::string name; // from the description
	std::string category;
	bool described{false};
	bool complete{false}; // can be placed from the node bank, see editor_includes::complete()
	std::string issue; // what keeps the template out of the node bank
	std::shared_ptr<std::string> statement; // node bank template standing for the include
};

// the scenery templates available to the editor: every *.inc file in the scenery directory. the templates whose
// descriptions are complete are offered by the node bank
class editorIncludeBank
{
  public:
	// looks through the scenery directory for the templates, and reads their descriptions
	void scan();
	bool scanned() const
	{
		return m_scanned;
	}
	std::vector<include_entry> const &entries() const
	{
		return m_entries;
	}
	// indices of the entries offered by the node bank, ordered by category and name
	std::vector<std::size_t> const &ready() const
	{
		return m_ready;
	}
	// reads description of specified template again, after it was changed
	void update(std::string const &File);
	// changes with each change of the entries
	int revision() const
	{
		return m_revision;
	}

  private:
	void index();

	std::vector<include_entry> m_entries; // sorted by file name
	std::vector<std::size_t> m_ready;
	bool m_scanned{false};
	int m_revision{0};
};

extern editorIncludeBank EditorIncludes;
