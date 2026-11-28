/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// writing, reading and checking of E4D files

#include "model/e4d/e4d_model.h"

#include <string>
#include <vector>

namespace e4d {

struct write_options {
	bool compress{true}; // zstd for the sections where it saves at least an eighth
	int level{9};
};

// the file of the model. the model is written as it is; validate() it first
std::vector<std::byte> write(model const &Model, write_options const &Options = {});

struct read_options {
	bool verify_hash{false}; // the hash of the content, which costs a pass over the whole file
	bool keep_unknown{true}; // the sections the reader doesn't know, for the tools to write back
};

struct read_result {
	bool ok{false};
	std::string error; // what is wrong with the file, when not ok
};

// reads the file into the model and checks it with validate(). a file which fails leaves the model empty
read_result read(std::span<std::byte const> File, model &Model, read_options const &Options = {});

// the faults of the model, each a line of text; empty when it can be used. stops after Limit faults
std::vector<std::string> validate(model const &Model, std::size_t Limit = 16);

} // namespace e4d
