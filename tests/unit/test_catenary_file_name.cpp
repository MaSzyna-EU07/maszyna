#include "stdafx.h"
#include "doctest.h"

#include "editor/editorCatenaryFile.hpp"

// the names of the catenary files the editor makes (editor/editorCatenaryFile.hpp): the directive which includes the file is
// read as tokens, and the file is written into the folder of the scenery

TEST_CASE("catenary file name: what the directive and the folder of the scenery take") {
	CHECK(catenary_file_name_valid("linia8.scm"));
	CHECK(catenary_file_name_valid("siec/linia8.scm"));
	CHECK(catenary_file_name_valid("siec\\linia_8.scm"));
	// a space or a ';' splits the token: the directive would name another file
	CHECK_FALSE(catenary_file_name_valid("catenary linia 8.scm"));
	CHECK_FALSE(catenary_file_name_valid("linia\t8.scm"));
	CHECK_FALSE(catenary_file_name_valid("linia;8.scm"));
	CHECK_FALSE(catenary_file_name_valid("\"linia8.scm\""));
	// out of the folder of the scenery
	CHECK_FALSE(catenary_file_name_valid("..\\linia8.scm"));
	CHECK_FALSE(catenary_file_name_valid("siec/../../linia8.scm"));
	CHECK_FALSE(catenary_file_name_valid("/linia8.scm"));
	CHECK_FALSE(catenary_file_name_valid("c:\\linia8.scm"));
	CHECK_FALSE(catenary_file_name_valid("siec//linia8.scm"));
	CHECK_FALSE(catenary_file_name_valid("siec/"));
	CHECK_FALSE(catenary_file_name_valid(""));
}
