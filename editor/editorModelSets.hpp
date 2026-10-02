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

// user-defined sets of node templates (e.g. "deciduous trees", "bushes"), used by the editor brush,
// random insert and area fill. stored next to the user's ini files
struct editor_model_set
{
	int id{0}; // runtime identifier, stable while the editor runs (renames don't break references)
	std::string name;
	std::vector<std::string> templates; // node definitions, one per entry, same format as nodebank entries
	std::vector<std::string> labels; // short readable names of the templates, cached for the UI
};

class editorModelSets
{
  public:
	bool load();
	bool save() const;

	std::vector<editor_model_set> const &sets() const
	{
		return m_sets;
	}
	editor_model_set const *find(int Id) const;
	// all modifying methods save the sets right away
	// returns: id of the new set
	int create(std::string const &Name, std::vector<std::string> const &Templates = {});
	void remove(int Id);
	void rename(int Id, std::string const &Name);
	void add(int Id, std::vector<std::string> const &Templates);
	void erase(int Id, std::size_t Index);
	void clear(int Id);
	// name not used by any other set, built from specified base
	std::string unique_name(std::string const &Base, int IgnoreId = 0) const;
	// short readable name of a node template (model file and texture)
	static std::string label(std::string const &Template);

  private:
	editor_model_set *find_(int Id);

	std::vector<editor_model_set> m_sets;
	int m_nextid{1};
};

extern editorModelSets EditorModelSets;
