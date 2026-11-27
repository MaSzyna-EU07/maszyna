/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// headless tools for the E3D1 models with shared geometry (see model/geometrypack.h), run with the null renderer:
//   -e3dpack <list> <input root> <output root> [-threshold <bytes>] [-nopack]
//       converts the E3D0 files of the list (paths relative to the input root) to E3D1 under the output root, adding
//       their geometry blocks to geometry.e3dpack of the vehicle family folder (dynamic/<country>/<vehicle>) or of the
//       models folder (models/<category>). a file which can't be converted is copied as it is
//   -e3dverify <list> <root A> <root B>
//       loads every model of the list from both roots through TModel3d and compares the geometry of the sub-models
//   -e3dbench <list> <root> <repeats> [-csv <file>] [-label <text>]
//       loads every model of the list through TModel3d::LoadFromFile and times it; the geometry is handed to the
//       null renderer, so the upload to the graphics card (the same for both formats) isn't part of it

#include "stdafx.h"

#include "model/Model3d.h"
#include "model/geometrypack.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/wholefile.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <span>
#include <thread>

namespace {

using clock_type = std::chrono::steady_clock;

double seconds_since(clock_type::time_point const Start)
{
	return std::chrono::duration<double>(clock_type::now() - Start).count();
}

std::vector<std::string> read_list(std::string const &File)
{
	std::vector<std::string> list;
	std::ifstream input(File);
	std::string line;
	while (std::getline(input, line))
	{
		while (false == line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t'))
			line.pop_back();
		if (line.empty() || line[0] == '#')
			continue;
		std::ranges::replace(line, '\\', '/');
		list.push_back(line);
	}
	return list;
}

// the command line arguments
using arguments = std::span<char *const>;

std::string option(arguments const Args, std::size_t const First, std::string_view const Name, std::string_view const Default = {})
{
	for (std::size_t i = First; i + 1 < Args.size(); ++i)
		if (Name == Args[i])
			return Args[i + 1];
	return std::string(Default);
}
bool flag(arguments const Args, std::size_t const First, std::string_view const Name)
{
	for (std::size_t i = First; i < Args.size(); ++i)
		if (Name == Args[i])
			return true;
	return false;
}

std::filesystem::path path_of(std::string const &Root, std::string const &Relative)
{
	return e3d::utf8_path(Root) / e3d::utf8_path(Relative);
}

// the folder of the pack a model goes with: the vehicle (dynamic/<country>/<vehicle>) or the models category (models/<category>)
std::filesystem::path family_folder(std::string const &Relative)
{
	auto const path{e3d::utf8_path(Relative)};
	std::vector<std::filesystem::path> parts(path.begin(), path.end());
	std::size_t depth{false == parts.empty() && parts[0] == "dynamic" ? 3u : 2u};
	depth = std::min(depth, parts.size() - 1); // a folder, not the file itself
	std::filesystem::path folder;
	for (std::size_t i = 0; i < depth; ++i)
		folder /= parts[i];
	return folder;
}

bool is_dynamic(std::string const &Relative)
{
	return Relative.rfind("dynamic/", 0) == 0;
}

// the name LoadFromFile() is given, without the extension
std::string model_name(std::string const &Root, std::string const &Relative)
{
	auto const u8name{path_of(Root, Relative).generic_u8string()};
	std::string name(u8name.begin(), u8name.end());
	if (name.size() > 4 && (name.ends_with(".e3d") || name.ends_with(".E3D")))
		name.resize(name.size() - 4);
	return name;
}

// the log is drained by its service in the background, as in the simulator
class log_session
{
  public:
	explicit log_session(int const Mode)
	{
		Global.iWriteLogEnabled = Mode;
		m_thread = std::jthread(LogService);
	}
	~log_session()
	{
		StopLogService();
		m_thread.join();
	}

  private:
	std::jthread m_thread;
};

void reset_renderer()
{
	// a fresh null renderer, which lets go of the geometry of the models loaded before
	GfxRenderer = gfx_renderer_factory::get_instance()->create("null");
}

std::uint64_t size_of(std::filesystem::path const &Path)
{
	std::error_code error;
	auto const size{std::filesystem::file_size(Path, error)};
	return error ? 0 : size;
}

double megabytes(std::uint64_t const Bytes)
{
	return static_cast<double>(Bytes) / 1e6;
}

// the geometry of a sub-model kept by the renderer, empty for one without it
template <typename Array, typename Getter>
Array const &geometry_of(TSubModel const &Submodel, Getter const &Get)
{
	static Array const empty;
	return Submodel.m_geometry.handle != null_handle ? Get(Submodel.m_geometry.handle) : empty;
}

// the sub-models as both models have them: empty if they are the same, what differs otherwise.
// the vertices and indices compared are added to the counts
std::string submodel_difference(TSubModel const &Left, TSubModel const &Right, int const Index, std::uint64_t &Vertices, std::uint64_t &Indices)
{
	auto const vertices = [](gfx::geometry_handle const &H) -> gfx::vertex_array const & { return GfxRenderer->Vertices(H); };
	auto const indices = [](gfx::geometry_handle const &H) -> gfx::index_array const & { return GfxRenderer->Indices(H); };
	auto const userdata = [](gfx::geometry_handle const &H) -> gfx::userdata_array const & { return GfxRenderer->UserData(H); };
	auto const &va{geometry_of<gfx::vertex_array>(Left, vertices)};
	auto const &vb{geometry_of<gfx::vertex_array>(Right, vertices)};
	auto const &ia{geometry_of<gfx::index_array>(Left, indices)};
	auto const &ib{geometry_of<gfx::index_array>(Right, indices)};
	auto const &ua{geometry_of<gfx::userdata_array>(Left, userdata)};
	auto const &ub{geometry_of<gfx::userdata_array>(Right, userdata)};
	Vertices += va.size();
	Indices += ia.size();
	if (false == e3d::same_bits(va, vb))
		return std::format("vertices of sub-model {} ({})", Index, Left.pName);
	if (ia != ib)
		return std::format("indices of sub-model {} ({})", Index, Left.pName);
	if (false == e3d::same_bits(ua, ub))
		return std::format("user data of sub-model {} ({})", Index, Left.pName);
	bool const samematrix{(Left.fMatrix == nullptr) == (Right.fMatrix == nullptr) &&
	                      (Left.fMatrix == nullptr || std::memcmp(Left.fMatrix->readArray(), Right.fMatrix->readArray(), 16 * sizeof(float)) == 0)};
	if (Left.pName != Right.pName || Left.eType != Right.eType || Left.iFlags != Right.iFlags || Left.m_geometry.vertex_count != Right.m_geometry.vertex_count ||
	    Left.m_geometry.index_count != Right.m_geometry.index_count || Left.f4Diffuse != Right.f4Diffuse || Left.m_materialname != Right.m_materialname || false == samematrix)
		return std::format("properties of sub-model {} ({})", Index, Left.pName);
	return {};
}

// the model of the list loaded from both roots: empty if they are the same, what differs otherwise
std::string model_difference(std::string const &Roota, std::string const &Rootb, std::string const &Relative, std::size_t &Submodels, std::uint64_t &Vertices, std::uint64_t &Indices)
{
	auto a{std::make_unique<TModel3d>()};
	auto b{std::make_unique<TModel3d>()};
	bool const loadeda{a->LoadFromFile(model_name(Roota, Relative), is_dynamic(Relative))};
	bool const loadedb{b->LoadFromFile(model_name(Rootb, Relative), is_dynamic(Relative))};
	if (loadeda != loadedb)
		return std::format("loaded {} vs {}", static_cast<int>(loadeda), static_cast<int>(loadedb));
	if (a->SubModelsCount() != b->SubModelsCount())
		return std::format("sub-model count {} vs {}", a->SubModelsCount(), b->SubModelsCount());
	std::string problem;
	for (int i = 0; i < a->SubModelsCount(); ++i)
	{
		problem = submodel_difference(a->Root[i], b->Root[i], i, Vertices, Indices);
		if (false == problem.empty())
			break;
	}
	Submodels += static_cast<std::size_t>(a->SubModelsCount());
	return problem;
}

} // namespace

int e3d1_pack_standalone(int Argc, char *Argv[])
{
	if (Argc < 5)
	{
		std::printf("usage: -e3dpack <list> <input root> <output root> [-threshold <bytes>] [-nopack]\n");
		return 2;
	}
	arguments const args(Argv, static_cast<std::size_t>(Argc));
	log_session log(2);
	auto list{read_list(Argv[2])};
	// the order of the blocks in the packs follows the order of the files
	std::ranges::sort(list);
	std::string const inroot{Argv[3]};
	std::string const outroot{Argv[4]};
	e3d::writer_options options;
	options.inline_threshold = static_cast<std::size_t>(std::stoull(option(args, 5, "-threshold", "4096")));
	options.use_pack = (false == flag(args, 5, "-nopack"));

	auto const start{clock_type::now()};
	double readtime{0};
	double splittime{0};
	double writetime{0};
	double hashtime{0};
	std::uint64_t inbytes{0};
	std::uint64_t outbytes{0};
	std::uint64_t newbytes{0};
	std::size_t converted{0};
	std::size_t copied{0};
	std::size_t blocks{0};
	std::size_t inlineblocks{0};
	std::size_t packedblocks{0};
	std::size_t newblocks{0};
	std::map<std::filesystem::path, std::filesystem::path> packs;
	for (auto const &relative : list)
	{
		auto const input{path_of(inroot, relative)};
		auto const output{path_of(outroot, relative)};
		auto timer{clock_type::now()};
		// read the way the game reads a model file, in one go
		auto const file{read_whole_file(input.string())};
		std::string_view const data(file.data.get(), file.size);
		if (data.empty() || data.size() != size_of(input))
		{
			std::printf("FAIL read %s\n", relative.c_str());
			continue;
		}
		readtime += seconds_since(timer);
		inbytes += data.size();

		timer = clock_type::now();
		e3d::e3d1_content content;
		std::string error;
		bool const split{e3d::split_e3d0(reinterpret_cast<unsigned char const *>(data.data()), data.size(), content, error)};
		splittime += seconds_since(timer);

		timer = clock_type::now();
		if (split)
		{
			options.pack = e3d::utf8_path(outroot) / family_folder(relative) / "geometry.e3dpack";
			packs.try_emplace(options.pack.lexically_normal(), options.pack);
			try
			{
				auto const stats{e3d::write_e3d1(output, content, options)};
				++converted;
				outbytes += stats.file_bytes;
				blocks += stats.blocks;
				inlineblocks += stats.inline_blocks;
				packedblocks += stats.packed_blocks;
				newblocks += stats.new_blocks;
				newbytes += stats.new_bytes;
				hashtime += stats.hash_seconds;
			}
			catch (std::exception const &exception)
			{
				std::printf("FAIL write %s: %s\n", relative.c_str(), exception.what());
				return 1;
			}
		}
		else
		{
			// left as it is
			std::printf("COPY %s: %s\n", relative.c_str(), error.c_str());
			std::error_code ec;
			std::filesystem::create_directories(output.parent_path(), ec);
			std::ofstream copy(output, std::ios::binary);
			copy.write(data.data(), static_cast<std::streamsize>(data.size()));
			++copied;
			outbytes += data.size();
		}
		writetime += seconds_since(timer);
	}
	auto timer{clock_type::now()};
	e3d::flush_pack_writers();
	writetime += seconds_since(timer);
	auto const total{seconds_since(start)};
	std::uint64_t packbytes{0};
	for (auto const &[folder, pack] : packs)
		packbytes += size_of(pack);

	std::printf("files %zu converted %zu copied %zu packs %zu\n", list.size(), converted, copied, packs.size());
	std::printf("input %.1f MB, output files %.1f MB, packs %.1f MB, output total %.1f MB (%.1f%%)\n", megabytes(inbytes), megabytes(outbytes), megabytes(packbytes),
	            megabytes(outbytes + packbytes), inbytes > 0 ? 100.0 * static_cast<double>(outbytes + packbytes) / static_cast<double>(inbytes) : 0.0);
	std::printf("blocks %zu inline %zu packed %zu new in pack %zu (%.1f MB)\n", blocks, inlineblocks, packedblocks, newblocks, megabytes(newbytes));
	std::printf("time total %.2f s: read %.2f s, split %.2f s, write %.2f s (of which hashing %.2f s, %.0f MB/s)\n", total, readtime, splittime, writetime, hashtime,
	            hashtime > 0 ? megabytes(inbytes) / hashtime : 0.0);
	std::fflush(stdout);
	return 0;
}

int e3d1_verify_standalone(int Argc, char *Argv[])
{
	if (Argc < 5)
	{
		std::printf("usage: -e3dverify <list> <root A> <root B>\n");
		return 2;
	}
	arguments const args(Argv, static_cast<std::size_t>(Argc));
	log_session log(flag(args, 5, "-quiet") ? 0 : 2);
	auto const list{read_list(Argv[2])};
	std::string const roota{Argv[3]};
	std::string const rootb{Argv[4]};
	std::size_t same{0};
	std::size_t different{0};
	std::size_t submodels{0};
	std::uint64_t vertices{0};
	std::uint64_t indices{0};
	reset_renderer();
	for (auto const &relative : list)
	{
		auto const problem{model_difference(roota, rootb, relative, submodels, vertices, indices)};
		reset_renderer();
		if (problem.empty())
			++same;
		else
		{
			++different;
			std::printf("DIFF %s: %s\n", relative.c_str(), problem.c_str());
		}
	}
	std::printf("verify: %zu models the same, %zu different; %zu sub-models, %llu vertices, %llu indices compared\n", same, different, submodels, static_cast<unsigned long long>(vertices),
	            static_cast<unsigned long long>(indices));
	std::fflush(stdout);
	return different == 0 ? 0 : 1;
}

int e3d1_bench_standalone(int Argc, char *Argv[])
{
	if (Argc < 5)
	{
		std::printf("usage: -e3dbench <list> <root> <repeats> [-csv <file>] [-label <text>]\n");
		return 2;
	}
	arguments const args(Argv, static_cast<std::size_t>(Argc));
	log_session log(0);
	auto const list{read_list(Argv[2])};
	std::string const root{Argv[3]};
	int const repeats{std::max(1, std::stoi(Argv[4]))};
	auto const csvfile{option(args, 5, "-csv")};
	auto const label{option(args, 5, "-label", "bench")};
	std::FILE *csv{csvfile.empty() ? nullptr : std::fopen(csvfile.c_str(), "a")};

	std::vector<std::uint64_t> filesizes;
	for (auto const &relative : list)
		filesizes.push_back(size_of(path_of(root, relative)));

	std::vector<double> totals;
	std::vector<std::vector<double>> permodel(list.size());
	reset_renderer();
	for (int repeat = 0; repeat < repeats; ++repeat)
	{
		// every pass opens the packs anew, like a new run of the simulator
		e3d::pack_reader::close_all();
		double total{0};
		std::uint64_t bytes{0};
		std::size_t failed{0};
		for (std::size_t i = 0; i < list.size(); ++i)
		{
			auto const packbytes{e3d::pack_bytes_read()};
			auto model{std::make_unique<TModel3d>()};
			auto const start{clock_type::now()};
			bool const loaded{model->LoadFromFile(model_name(root, list[i]), is_dynamic(list[i]))};
			auto const elapsed{seconds_since(start) * 1000.0};
			auto const read{filesizes[i] + (e3d::pack_bytes_read() - packbytes)};
			model.reset();
			reset_renderer();
			total += elapsed;
			bytes += read;
			failed += loaded ? 0 : 1;
			permodel[i].push_back(elapsed);
			if (csv != nullptr)
				std::fprintf(csv, "%s,%d,%s,%.4f,%llu,%d\n", label.c_str(), repeat, list[i].c_str(), elapsed, static_cast<unsigned long long>(read), loaded ? 1 : 0);
		}
		totals.push_back(total);
		std::printf("%s pass %d: %zu models, %.1f ms, %.1f MB read, %zu failed\n", label.c_str(), repeat, list.size(), total, megabytes(bytes), failed);
		std::fflush(stdout);
	}
	if (csv != nullptr)
		std::fclose(csv);
	auto median = [](std::vector<double> Values) {
		if (Values.empty())
			return 0.0;
		std::ranges::sort(Values);
		auto const middle{Values.size() / 2};
		return (Values.size() % 2) ? Values[middle] : 0.5 * (Values[middle - 1] + Values[middle]);
	};
	std::vector<double> modelmedians;
	for (auto const &times : permodel)
		modelmedians.push_back(median(times));
	std::printf("%s summary: median pass %.1f ms, median model %.3f ms\n", label.c_str(), median(totals), median(modelmedians));
	std::fflush(stdout);
	return 0;
}

// the tool the first argument names: -e3dpack, -e3dverify or -e3dbench
int e3d1_tools_standalone(int Argc, char *Argv[])
{
	std::string_view const mode(Argv[1]);
	if (mode == "-e3dpack")
		return e3d1_pack_standalone(Argc, Argv);
	if (mode == "-e3dverify")
		return e3d1_verify_standalone(Argc, Argv);
	return e3d1_bench_standalone(Argc, Argv);
}
