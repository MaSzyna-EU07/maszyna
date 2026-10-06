#include "stdafx.h"

#include "scene/sceneterrain.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

// converts text terrain file to binary format. returns: 0 on success, 1 otherwise
int export_btf_standalone(std::string in, std::string out)
{
	if (in.empty())
	{
		std::printf("usage: -btf -s <text terrain file> [-o <binary terrain file>]\n");
		return 1;
	}
	replace_slashes(in);
	replace_slashes(out);
	if (false == FileExists(in) && FileExists(Global.asCurrentSceneryPath + in))
	{
		// the file can be given the way the sceneries refer to it
		in = Global.asCurrentSceneryPath + in;
	}
	if (out.empty())
	{
		// by default the binary file goes where the sceneries expect to find it, next to the text
		out = in;
		erase_extension(out);
		out += ".btf";
	}
	std::string message;
	auto const result{scene::terrain_file::convert(in, out, &message)};
	// NOTE: the log service doesn't run in this mode, the outcome is reported directly
	if (result)
	{
		std::printf("%s -> %s: %s\n", in.c_str(), out.c_str(), message.c_str());
	}
	else
	{
		std::printf("%s: conversion failed, %s\n", in.c_str(), message.c_str());
	}
	return (result ? 0 : 1);
}
