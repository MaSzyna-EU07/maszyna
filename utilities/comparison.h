/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

enum class comparison_operator {
    equal, // ==
    not_equal, // !=
    lt, // <
    gt, // >
    lt_eq, // <=
    gt_eq // >=
};

enum class comparison_pass {
    all, // &&
    any, // ||
    none // !
};

template <typename Type_>
bool compare( Type_ const Left, Type_ const Right, comparison_operator const Operator ) {
    switch( Operator ) {
        using enum comparison_operator;
        case equal:     { return Left == Right; }
        case not_equal: { return Left != Right; }
        case lt:     { return Left < Right; }
        case gt:     { return Left > Right; }
        case lt_eq:  { return Left <= Right; }
        case gt_eq:  { return Left >= Right; }
        default:                          { return false; }
    }
}

inline
std::string
to_string( comparison_operator const Operator ) {
    switch( Operator ) {
        using enum comparison_operator;
        case equal:     { return "=="; }
        case not_equal: { return "!="; }
        case lt:     { return "<"; }
        case gt:     { return ">"; }
        case lt_eq:  { return "<="; }
        case gt_eq:  { return ">"; }
        default:                          { return "??"; }
    }
}

inline
comparison_pass
comparison_pass_from_string( std::string const &Input ) {
         using enum comparison_pass;
         if( Input == "all" ) { return all; }
    else if( Input == "any" ) { return any; }
    else if( Input == "none" ) { return none; }

    return all; // legacy default
}

inline
comparison_operator
comparison_operator_from_string( std::string const &Input ) {
         using enum comparison_operator;
         if( Input == "==" ) { return equal; }
    else if( Input == "!=" ) { return not_equal; }
    else if( Input == "<" )  { return lt; }
    else if( Input == ">" )  { return gt; }
    else if( Input == "<=" ) { return lt_eq; }
    else if( Input == ">=" ) { return gt_eq; }

    return equal; // legacy default
}

//---------------------------------------------------------------------------

