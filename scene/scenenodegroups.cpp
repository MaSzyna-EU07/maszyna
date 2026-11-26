/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/scenenodegroups.h"

#include "world/Event.h"
#include "world/MemCell.h"

#include "model/AnimModel.h"
#include "widgets/map_objects.h"

namespace scene {

node_groups Groups;

void update_map() {
	map::prepare_objects();
}

// requests creation of a new node group. returns: handle to the group
scene::group_handle
node_groups::create( bool const Separate ) {

    if( false == Separate ) {
        m_activegroup.push( create_handle() );
        return handle();
    }
    // a handle no other group has, see make()
    auto group { std::max<group_handle>( 2, static_cast<group_handle>( m_groupmap.size() + 1 ) ) };
    while( m_groupmap.find( group ) != m_groupmap.end() ) {
        ++group;
    }
    // NOTE: registered before the nodes come, so that a group nested in this one doesn't get the same handle
    m_groupmap[ group ];
    m_activegroup.push( group );

    return handle();
}

// indicates creation of current group ended. returns: handle to the parent group or null_handle if group stack is empty
scene::group_handle
node_groups::close()
{
    if( false == m_activegroup.empty() ) {

        auto const closinggroup { m_activegroup.top() };
        m_activegroup.pop();
        // if the completed group holds only one item and there's no chance more items will be added, disband it
        if( true == m_activegroup.empty()
         || m_activegroup.top() != closinggroup ) {

            auto lookup { m_groupmap.find( closinggroup ) };
            if( lookup != m_groupmap.end()
             && lookup->second.nodes.size() + lookup->second.events.size() <= 1 ) {

				erase( lookup );
            }
        }
    }

    return handle();
}

// returns current active group, or null_handle if group stack is empty
group_handle
node_groups::handle() const {

    return m_activegroup.empty() ? null_handle : m_activegroup.top();
}

// places provided node in specified group
void
node_groups::insert( scene::group_handle const Group, scene::basic_node *Node ) {

    // TBD, TODO: automatically unregister the node from its current group?
    auto const previousgroup { Node->group() };
    Node->group( Group );

    if( Group == null_handle ) { return; }

    auto &nodesequence { m_groupmap[ Group ].nodes };
    // a node which wasn't a member of the group can't be on its list, which saves the search through the list.
    // it matters as a group can be large, the root group of a scenery holds nearly all its nodes
    if( previousgroup != Group
     || std::find( std::begin( nodesequence ), std::end( nodesequence ), Node ) == std::end( nodesequence ) ) {
        // don't add the same node twice
        nodesequence.emplace_back( Node );
    }
}

// places provided event in specified group
void
node_groups::insert( scene::group_handle const Group, basic_event *Event ) {

    // TBD, TODO: automatically unregister the event from its current group?
    auto const previousgroup { Event->group() };
    Event->group( Group );

    if( Group == null_handle ) { return; }

    auto &eventsequence { m_groupmap[ Group ].events };
    // same as with the nodes, the list is searched only for an event which already was a member of the group
    if( previousgroup != Group
     || std::find( std::begin( eventsequence ), std::end( eventsequence ), Event ) == std::end( eventsequence ) ) {
        // don't add the same node twice
        eventsequence.emplace_back( Event );
    }
}

// sends basic content of the class in legacy (text) format to provided stream
void
node_groups::export_as_text( std::ostream &Output, bool Dirty ) const {
    for( auto const &group : m_groupmap ) {
		bool any = false;
        for( auto *node : group.second.nodes ) {
			if (node->dirty() != Dirty)
				continue;
            // HACK: auto-generated memory cells aren't exported, so we check for this
            // TODO: is_exportable as basic_node method
            if( typeid(*node) == typeid(TMemCell)
             && false == static_cast<TMemCell *>(node)->is_exportable ) {
                continue;
            }

			if (!any)
				Output << "group\n";
			any = true;

            node->export_as_text( Output );
        }
        for( auto *event : group.second.events ) {
			if (Dirty)
				continue;

			if (!any)
				Output << "group\n";
			any = true;

            event->export_as_text( Output );
        }
		if (any)
			Output << "endgroup\n";
    }
}

// removes specified group from the group list and group information from the group's nodes
void
node_groups::erase( group_map::const_iterator Group ) {

    for( auto *node : Group->second.nodes ) {
        node->group( null_handle );
    }
    for( auto *event : Group->second.events ) {
        event->group( null_handle );
    }
    m_groupmap.erase( Group );
}

group_handle
node_groups::make( std::vector<scene::basic_node *> const &Nodes ) {

    // a handle no other group has, past the first one, which the editor treats as no group at all
    auto handle { std::max<group_handle>( 2, static_cast<group_handle>( m_groupmap.size() + 1 ) ) };
    while( m_groupmap.find( handle ) != m_groupmap.end() ) {
        ++handle;
    }
    for( auto *node : Nodes ) {
        leave( node );
        m_loose.erase( node );
        insert( handle, node );
    }
    m_rewrite.insert( handle );
    return handle;
}

void
node_groups::dissolve( scene::group_handle const Group ) {

    auto const lookup { m_groupmap.find( Group ) };
    if( lookup == m_groupmap.end() ) { return; }
    for( auto *node : lookup->second.nodes ) {
        m_loose.insert( node );
    }
    m_rewrite.erase( Group );
    erase( lookup );
}

void
node_groups::remove( scene::basic_node *Node ) {

    leave( Node );
    m_loose.erase( Node );
}

void
node_groups::leave( scene::basic_node *Node ) {

    auto const group { Node->group() };
    Node->group( null_handle );
    auto const lookup { m_groupmap.find( group ) };
    if( lookup == m_groupmap.end() ) { return; }
    auto &nodes { lookup->second.nodes };
    nodes.erase( std::remove( std::begin( nodes ), std::end( nodes ), Node ), std::end( nodes ) );
    if( nodes.empty() && lookup->second.events.empty() ) {
        // a group made in the editor goes with its last node
        m_rewrite.erase( group );
        m_groupmap.erase( lookup );
    }
}

bool
node_groups::relocating( scene::basic_node const *Node ) const {

    return ( m_loose.count( Node ) != 0 ) || ( m_rewrite.count( Node->group() ) != 0 );
}

std::vector<scene::group_handle>
node_groups::rewritten() const {

    return { std::begin( m_rewrite ), std::end( m_rewrite ) };
}

std::vector<scene::basic_node *>
node_groups::loose() const {

    std::vector<scene::basic_node *> nodes;
    for( auto const *node : m_loose ) {
        nodes.emplace_back( const_cast<scene::basic_node *>( node ) );
    }
    return nodes;
}

void
node_groups::saved() {

    m_rewrite.clear();
    m_loose.clear();
}

// creates handle for a new group
group_handle
node_groups::create_handle() {
    // NOTE: for simplification nested group structure are flattened
    return m_activegroup.empty() ? m_groupmap.size() + 1 : // new group isn't created until node registration
	                               m_activegroup.top();
}

} // scene

//---------------------------------------------------------------------------
