/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scene/scenenode.h"

namespace scene {

// TODO: move the snapshot to history stack
struct node_snapshot {

    basic_node *node;
    std::string data;

    node_snapshot( basic_node *Node ) :
        node( Node ) {
        if( Node != nullptr ) {
            Node->export_as_text( data ); } };
};

inline bool operator==( node_snapshot const &Left, node_snapshot const &Right ) { return Left.node == Right.node && Left.data == Right.data; }
inline bool operator!=( node_snapshot const &Left, node_snapshot const &Right ) { return !(Left == Right); }

class basic_editor {

public:
// methods
    void
        translate( basic_node *Node, glm::dvec3 const &Location, bool Snaptoground );
    void
        translate( basic_node *Node, float Offset );
    void
        rotate( basic_node *Node, glm::vec3 const &Angle, float Quantization );

private:
// methods
    void
        translate_node( basic_node *Node, glm::dvec3 const &Location );
    void
        translate_node( basic_node *Node, float Offset );
	static void
        translate_instance( TAnimModel *Instance, glm::dvec3 const &Location );
	static void
        translate_instance( TAnimModel *Instance, float Offset );
	static void
        translate_memorycell( TMemCell *Memorycell, glm::dvec3 const &Location );
	static void
        translate_memorycell( TMemCell *Memorycell, float Offset );
    void
        rotate_node( basic_node *Node, glm::vec3 const &Angle );
	static void
        rotate_instance( TAnimModel *Instance, glm::vec3 const &Angle );
};

} // scene

//---------------------------------------------------------------------------
