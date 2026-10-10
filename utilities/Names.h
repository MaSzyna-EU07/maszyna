/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <unordered_map>
#include <string>
#include <algorithm>

template <typename Type_>
class basic_table {

public:
// destructor
    ~basic_table() {
        for( auto *item : m_items ) {
			if (item)
				delete item; } }
// methods
    // adds provided item to the collection. returns: true if there's no duplicate with the same name, false otherwise
    bool
        insert( Type_ *Item, std::string itemname ) {
            m_items.emplace_back( Item );
            if( true == itemname.empty() || itemname == "none" ) {
                return true;
            }
            auto const itemhandle { m_items.size() - 1 };
            // add item name to the map
            auto mapping = m_itemmap.emplace( std::move( itemname ), itemhandle );
            if( true == mapping.second ) {
                return true;
            }
            // item with this name already exists; update mapping to point to the new one, for backward compatibility
            mapping.first->second = itemhandle;
            return false; }
	bool insert (Type_ *Item)
	{
		return insert(Item, Item->name());
	}
    // changes the name specified item is found by; empty text or "none" leaves the item without one.
    // NOTE: the name held by the item itself is for the caller to change afterwards.
    // returns: false if the name is in use by another item or the item isn't in the collection, true otherwise
    bool
        rename( Type_ const *Item, std::string const &Name ) {
            auto const named { false == Name.empty() && Name != "none" };
            if( named ) {
                auto const taken { m_itemmap.find( Name ) };
                if( taken != m_itemmap.end()
                 && m_items[ taken->second ] != nullptr
                 && m_items[ taken->second ] != Item ) {
                    return false; } }
            // the current name leads to the item unless the item has none, or shares it with an item added later
            auto itemhandle { m_items.size() };
            auto const current { m_itemmap.find( Item->name() ) };
            if( current != m_itemmap.end()
             && m_items[ current->second ] == Item ) {
                itemhandle = current->second;
                m_itemmap.erase( current ); }
            else {
                auto const lookup { std::find( m_items.begin(), m_items.end(), Item ) };
                if( lookup == m_items.end() ) {
                    return false; }
                itemhandle = static_cast<std::size_t>( std::distance( m_items.begin(), lookup ) ); }
            if( named ) {
                m_itemmap[ Name ] = itemhandle; }
            return true; }
	void purge (std::string const &Name)
	{
		auto lookup = m_itemmap.find( Name );
		if (lookup == m_itemmap.end())
			return;
		delete m_items[lookup->second];

		detach(Name);
	}
	void detach (std::string const &Name)
	{
		auto lookup = m_itemmap.find( Name );
		if (lookup == m_itemmap.end())
			return;

		m_items[lookup->second] = nullptr;
		// TBD, TODO: remove from m_items?

		m_itemmap.erase(lookup);
	}
	uint32_t find_id( std::string const &Name) const {
		auto lookup = m_itemmap.find( Name );
		return lookup != m_itemmap.end() ? lookup->second : -1;
	}
	void purge (Type_ *Item)
	{
		for (auto it = m_items.begin(); it != m_items.end(); it++) {
			if (*it == Item) {
				delete *it;
				*it = nullptr;
				return;
			}
		}
	}
    // locates item with specified name. returns pointer to the item, or nullptr
    Type_ *
        find( std::string const &Name ) const {
            auto lookup = m_itemmap.find( Name );
            return lookup != m_itemmap.end() ? m_items[lookup->second] : nullptr; }

protected:
// types
    using type_sequence = std::deque<Type_ *>;
    using index_map = std::unordered_map<std::string, std::size_t>;
// members
    type_sequence m_items;
    index_map m_itemmap;

public:
    // data access
    type_sequence &
        sequence() {
            return m_items; }
    type_sequence const &
        sequence() const {
            return m_items; }

};
