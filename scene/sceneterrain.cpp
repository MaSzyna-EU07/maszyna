/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/sceneterrain.h"

#include "scene/scene.h"
#include "rendering/renderer.h"
#include "rendering/nullrenderer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"

#include <bit>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unordered_map>

/*
binary terrain file layout, all values little endian:

    uint32   file id ('EU07')
    uint32   format id ('BTF1')
    uint64   length of the source text file
    uint32   checksum (crc-32) of the source text file
    uint32   section size, uint32 cell size, uint32 number of sections along the side of the region
    uint32   material count
    uint32   section count
    uint64   vertex count
    material names, null-terminated
    section directory, per section:
        uint32 section index, uint64 offset of the section data in the file, uint32 size of the section data, float32 bounding radius of the section
    section data, per section:
        uint32   block id ('BTFS')
        uint32   section index
        uint32   shape count
        per shape:
            uint32   material index
            uint32   index of the section cell the shape is located in
            float64  squared minimum and maximum visibility range
            float32  x12 diffuse, ambient and specular colours
            float64  x3 centre, float32 bounding radius
            uint32   vertex count
            per vertex: float32 x3 position relative to the centre of the section, float32 x3 normal, float32 x2 texture coordinates

the shapes are the ones the scenery loader would put in the section cells for the same text. where the loader would merge them
further into section-wide pieces depends on the materials, which are known only at run time, so that part is left for loading.
*/

namespace scene
{

// defined along with the scenery loader code which filters out legacy switch trackbeds
extern std::vector<std::string> switchtrackbedtextures;

// data shared by the pieces of a binary terrain file put to use
struct terrain_source
{
	struct material
	{
		material_handle handle{null_handle};
		bool translucent{false};
		bool clamps{false}; // texture coordinates are kept off the edges of the texture
		bool clampt{false};
		bool skip{false}; // geometry using the material isn't loaded
	};
	std::string file;
	std::vector<material> materials;
};

namespace
{

std::uint32_t const EU07_FILEHEADER{MAKE_ID4('E', 'U', '0', '7')};
std::uint32_t const EU07_FILEVERSION_TERRAIN{MAKE_ID4('B', 'T', 'F', '1')};
std::uint32_t const EU07_TERRAINBLOCK{MAKE_ID4('B', 'T', 'F', 'S')};
std::size_t const EU07_TERRAINVERTEXSIZE{8 * sizeof(float)};

struct file_header
{
	std::uint64_t sourcesize{0};
	std::uint32_t sourcechecksum{0};
	std::uint32_t materialcount{0};
	std::uint32_t sectioncount{0};
	std::uint64_t vertexcount{0};
};

// adds value to the data in the byte order of the file
template <typename Type_> void put(std::string &Output, Type_ Value)
{
	char bytes[sizeof(Type_)];
	std::memcpy(bytes, &Value, sizeof(Type_));
	if (std::endian::native != std::endian::little)
	{
		std::reverse(std::begin(bytes), std::end(bytes));
	}
	Output.append(bytes, sizeof(Type_));
}

// extracts values in the byte order of the file from a block of data, keeping track of its bounds
class data_reader
{
  public:
	data_reader(char const *Data, std::size_t const Size) : m_data(Data), m_size(Size) {}
	template <typename Type_> Type_ get()
	{
		Type_ value{};
		if (false == has(sizeof(Type_)))
		{
			m_good = false;
			m_position = m_size;
			return value;
		}
		char bytes[sizeof(Type_)];
		std::memcpy(bytes, m_data + m_position, sizeof(Type_));
		if (std::endian::native != std::endian::little)
		{
			std::reverse(std::begin(bytes), std::end(bytes));
		}
		std::memcpy(&value, bytes, sizeof(Type_));
		m_position += sizeof(Type_);
		return value;
	}
	std::string get_string()
	{
		std::string text;
		while (has(1) && m_data[m_position] != 0)
		{
			text += m_data[m_position++];
		}
		if (false == has(1))
		{
			m_good = false;
			return text;
		}
		++m_position; // terminator
		return text;
	}
	// extracts a series of values at once. NOTE: the caller is expected to check the data is there
	void get(float *Values, std::size_t const Count)
	{
		if (false == has(Count * sizeof(float)))
		{
			m_good = false;
			m_position = m_size;
			return;
		}
		if (std::endian::native == std::endian::little)
		{
			std::memcpy(Values, m_data + m_position, Count * sizeof(float));
			m_position += Count * sizeof(float);
		}
		else
		{
			for (std::size_t index{0}; index < Count; ++index)
			{
				Values[index] = get<float>();
			}
		}
	}
	void skip(std::size_t const Size)
	{
		if (false == has(Size))
		{
			m_good = false;
			m_position = m_size;
			return;
		}
		m_position += Size;
	}
	bool has(std::size_t const Size) const
	{
		return Size <= m_size - m_position;
	}
	bool good() const
	{
		return m_good;
	}

  private:
	char const *m_data;
	std::size_t m_size;
	std::size_t m_position{0};
	bool m_good{true};
};

// loads content of specified file into provided string. returns: true on success, false otherwise
bool read_file(std::string const &Filename, std::string &Data)
{
	std::ifstream input{Filename, std::ios::binary};
	if (input.fail())
	{
		return false;
	}
	auto const size{input.rdbuf()->pubseekoff(0, std::ios_base::end)};
	input.rdbuf()->pubseekoff(0, std::ios_base::beg);
	if (size < 0 || static_cast<std::uintmax_t>(size) >= Data.max_size())
	{
		return false;
	}
	Data.resize(static_cast<std::size_t>(size));
	input.read(Data.data(), static_cast<std::streamsize>(Data.size()));
	return static_cast<std::size_t>(std::max<std::streamsize>(0, input.gcount())) == Data.size();
}

// loads the fixed part of binary terrain file header. returns: true if the file can be used, false otherwise
bool read_header(std::istream &Input, file_header &Header)
{
	char bytes[48];
	Input.read(bytes, sizeof(bytes));
	if (Input.gcount() != static_cast<std::streamsize>(sizeof(bytes)))
	{
		return false;
	}
	data_reader data{bytes, sizeof(bytes)};
	if (data.get<std::uint32_t>() != EU07_FILEHEADER || data.get<std::uint32_t>() != EU07_FILEVERSION_TERRAIN)
	{
		// wrong file type
		return false;
	}
	Header.sourcesize = data.get<std::uint64_t>();
	Header.sourcechecksum = data.get<std::uint32_t>();
	// the content is arranged for specific layout of the scene, and can't be used with any other
	if (data.get<std::uint32_t>() != static_cast<std::uint32_t>(EU07_SECTIONSIZE) || data.get<std::uint32_t>() != static_cast<std::uint32_t>(EU07_CELLSIZE) ||
	    data.get<std::uint32_t>() != static_cast<std::uint32_t>(EU07_REGIONSIDESECTIONCOUNT))
	{
		return false;
	}
	Header.materialcount = data.get<std::uint32_t>();
	Header.sectioncount = data.get<std::uint32_t>();
	Header.vertexcount = data.get<std::uint64_t>();
	return data.good();
}

bool read_header(std::string const &Filename, file_header &Header)
{
	std::ifstream input{Filename, std::ios::binary};
	return (false == input.fail()) && read_header(input, Header);
}

// stand-in for the renderer used during conversion of text terrain. the conversion doesn't need
// anything of the materials but a way to tell them apart, so it goes by their names alone
class conversion_renderer : public null_renderer
{
  public:
	material_handle Fetch_Material(std::string const &Filename, bool const Loadnow = true) override
	{
		// the names are brought to the form the material manager would look them up by
		auto name{Filename};
		if (name.find("make:") != 0 && name.find("internal_src:") != 0)
		{
			if (contains(name, '|'))
			{
				name.erase(name.find('|'));
			}
			erase_extension(name);
			replace_slashes(name);
			erase_leading_slashes(name);
		}
		auto const lookup{m_lookup.find(name)};
		if (lookup != m_lookup.end())
		{
			return lookup->second;
		}
		m_materials.emplace_back(std::make_unique<opengl_material>());
		m_materials.back()->name = name;
		m_names.emplace_back(name);
		m_lookup.emplace(name, static_cast<material_handle>(m_materials.size()));
		return static_cast<material_handle>(m_materials.size());
	}
	IMaterial const *Material(material_handle const Material) const override
	{
		return (Material > 0 && static_cast<std::size_t>(Material) <= m_materials.size() ? m_materials[Material - 1].get() : IMaterial::null_material());
	}
	// names of the materials, in the order of their handles, starting with handle 1
	std::vector<std::string> const &names() const
	{
		return m_names;
	}

  private:
	std::vector<std::unique_ptr<opengl_material>> m_materials;
	std::vector<std::string> m_names;
	std::unordered_map<std::string, material_handle> m_lookup;
};

// puts the stand-in renderer in place of the real one for as long as the object lives.
// the conversion is done by the code which loads scenery geometry from text, and that code asks the global renderer for the materials
class conversion_scope
{
  public:
	conversion_scope() : m_trackbeds(Global.CreateSwitchTrackbeds)
	{
		auto renderer{std::make_unique<conversion_renderer>()};
		m_materials = renderer.get();
		m_renderer = std::move(renderer);
		std::swap(GfxRenderer, m_renderer);
		// the binary file holds everything the text does. geometry of legacy switch trackbeds is filtered out on load, as that depends on the settings
		Global.CreateSwitchTrackbeds = false;
	}
	~conversion_scope()
	{
		std::swap(GfxRenderer, m_renderer);
		Global.CreateSwitchTrackbeds = m_trackbeds;
	}
	conversion_scope(conversion_scope const &) = delete;
	conversion_scope &operator=(conversion_scope const &) = delete;
	std::vector<std::string> const &materials() const
	{
		return m_materials->names();
	}

  private:
	std::unique_ptr<gfx_renderer> m_renderer;
	conversion_renderer *m_materials{nullptr};
	bool m_trackbeds;
};

// collects what the loader needs to know about the material with specified name
terrain_source::material fetch_material(std::string const &Name)
{
	terrain_source::material material;
	material.handle = GfxRenderer->Fetch_Material(Name);
	// the appearance is determined from the assigned diffuse texture, the same way it's done for the geometry loaded from text
	auto const texturehandle = material.handle != null_handle ? GfxRenderer->Material(material.handle)->GetTexture(0) : null_handle;
	auto const &texture = texturehandle ? GfxRenderer->Texture(texturehandle) : *ITexture::null_texture();
	material.clamps = texturehandle ? contains(texture.get_traits(), 's') : false;
	material.clampt = texturehandle ? contains(texture.get_traits(), 't') : false;
	material.translucent = (texturehandle != null_handle) && contains(texture.get_name(), '@') && (true == texture.get_has_alpha());
	if (Global.CreateSwitchTrackbeds)
	{
		// geometry with blacklisted texture, part of old switch trackbed
		auto const materialname{GfxRenderer->Material(material.handle)->GetName()};
		material.skip =
		    std::any_of(std::begin(switchtrackbedtextures), std::end(switchtrackbedtextures), [&](std::string const &Texture) { return contains(materialname, Texture); });
	}
	return material;
}

} // namespace

// calculates checksum (crc-32) of provided data
std::uint32_t terrain_file::checksum(char const *Data, std::size_t Size)
{
	// lookup for 8 bytes of input at a time
	static auto const tables{[]() {
		std::array<std::array<std::uint32_t, 256>, 8> lookup{};
		for (std::uint32_t index{0}; index < 256; ++index)
		{
			auto value{index};
			for (auto bit{0}; bit < 8; ++bit)
			{
				value = (value & 1 ? 0xEDB88320u ^ (value >> 1) : value >> 1);
			}
			lookup[0][index] = value;
		}
		for (std::uint32_t index{0}; index < 256; ++index)
		{
			for (std::size_t table{1}; table < lookup.size(); ++table)
			{
				lookup[table][index] = (lookup[table - 1][index] >> 8) ^ lookup[0][lookup[table - 1][index] & 0xFF];
			}
		}
		return lookup;
	}()};

	auto const *data{reinterpret_cast<unsigned char const *>(Data)};
	std::uint32_t crc{0xFFFFFFFFu};
	while (Size >= 8)
	{
		auto const low{crc ^ (static_cast<std::uint32_t>(data[0]) | (static_cast<std::uint32_t>(data[1]) << 8) | (static_cast<std::uint32_t>(data[2]) << 16) | (static_cast<std::uint32_t>(data[3]) << 24))};
		crc = tables[7][low & 0xFF] ^ tables[6][(low >> 8) & 0xFF] ^ tables[5][(low >> 16) & 0xFF] ^ tables[4][low >> 24] ^ tables[3][data[4]] ^ tables[2][data[5]] ^ tables[1][data[6]] ^ tables[0][data[7]];
		data += 8;
		Size -= 8;
	}
	while (Size-- > 0)
	{
		crc = tables[0][(crc ^ *data++) & 0xFF] ^ (crc >> 8);
	}
	return crc ^ 0xFFFFFFFFu;
}

// converts provided content of a text terrain file to binary format and stores it in specified file. returns: true on success, false otherwise
bool terrain_file::write(std::string const &Text, std::string const &Binaryfile, std::string &Message)
{
	conversion_scope scope;
	auto region{std::make_unique<basic_region>()};
	scratch_data scratchpad{};

	{
		cParser input{Text, cParser::buffer_TEXT};
		// content of the files the text would include isn't covered by its checksum, so there can't be any
		input.expandIncludes = false;
		auto token{input.getToken<std::string>()};
		while (false == token.empty())
		{
			if (token == "node")
			{
				node_data nodedata;
				input.getTokens(4);
				input >> nodedata.range_max >> nodedata.range_min >> nodedata.name >> nodedata.type;
				if (nodedata.type != "triangles" && nodedata.type != "triangle_strip" && nodedata.type != "triangle_fan")
				{
					Message = "node of type \"" + nodedata.type + "\" (line " + std::to_string(input.Line()) + ") can't be a part of binary terrain";
					return false;
				}
				auto shape{shape_node().import(input, nodedata)};
				// the shapes are all sent to the section cells, which groups them the way it'd be done for translucent geometry.
				// whether they can be merged into section-wide pieces instead is decided when the file is loaded, see the layout notes
				shape.m_data.translucent = true;
				region->insert(shape, scratchpad, true);
			}
			else if (token == "origin")
			{
				glm::dvec3 offset;
				input.getTokens(3);
				input >> offset.x >> offset.y >> offset.z;
				scratchpad.location.offset.emplace(offset + (scratchpad.location.offset.empty() ? glm::dvec3() : scratchpad.location.offset.top()));
			}
			else if (token == "endorigin")
			{
				if (false == scratchpad.location.offset.empty())
				{
					scratchpad.location.offset.pop();
				}
			}
			else if (token == "rotate")
			{
				input.getTokens(3);
				input >> scratchpad.location.rotation.x >> scratchpad.location.rotation.y >> scratchpad.location.rotation.z;
			}
			else
			{
				Message = "\"" + token + "\" (line " + std::to_string(input.Line()) + ") can't be a part of binary terrain";
				return false;
			}
			token = input.getToken<std::string>();
		}
	}

	// sections holding any geometry
	std::vector<std::uint32_t> sections;
	for (std::size_t index{0}; index < region->m_sections.size(); ++index)
	{
		auto const *section{region->m_sections[index]};
		if (section == nullptr)
		{
			continue;
		}
		if (std::any_of(std::begin(section->m_cells), std::end(section->m_cells), [](basic_cell const &Cell) { return false == Cell.m_shapestranslucent.empty(); }))
		{
			sections.emplace_back(static_cast<std::uint32_t>(index));
		}
	}

	auto const temporaryfile{Binaryfile + ".tmp"};
	std::uint64_t shapecount{0};
	std::uint64_t vertexcount{0};
	{
		std::ofstream output{temporaryfile, std::ios::binary | std::ios::trunc};
		if (output.fail())
		{
			Message = "can't write to file \"" + temporaryfile + "\"";
			return false;
		}

		std::string data;
		put(data, EU07_FILEHEADER);
		put(data, EU07_FILEVERSION_TERRAIN);
		put(data, static_cast<std::uint64_t>(Text.size()));
		put(data, terrain_file::checksum(Text.data(), Text.size()));
		put(data, static_cast<std::uint32_t>(EU07_SECTIONSIZE));
		put(data, static_cast<std::uint32_t>(EU07_CELLSIZE));
		put(data, static_cast<std::uint32_t>(EU07_REGIONSIDESECTIONCOUNT));
		put(data, static_cast<std::uint32_t>(scope.materials().size()));
		put(data, static_cast<std::uint32_t>(sections.size()));
		auto const vertexcountposition{data.size()};
		put(data, vertexcount); // filled in when the count is known
		for (auto const &material : scope.materials())
		{
			data.append(material.c_str(), material.size() + 1);
		}
		auto const directoryposition{data.size()};
		// the directory is written once the section data is in place and its location known
		std::size_t const directoryentrysize{sizeof(std::uint32_t) + sizeof(std::uint64_t) + sizeof(std::uint32_t) + sizeof(float)};
		data.append(sections.size() * directoryentrysize, '\0');
		output.write(data.data(), static_cast<std::streamsize>(data.size()));
		std::uint64_t offset{data.size()};

		std::string directory;
		for (auto const sectionindex : sections)
		{
			auto *section{region->m_sections[sectionindex]};
			data.clear();
			put(data, EU07_TERRAINBLOCK);
			put(data, sectionindex);
			std::uint32_t sectionshapecount{0};
			for (auto const &cell : section->m_cells)
			{
				sectionshapecount += static_cast<std::uint32_t>(cell.m_shapestranslucent.size());
			}
			put(data, sectionshapecount);
			for (std::size_t cellindex{0}; cellindex < section->m_cells.size(); ++cellindex)
			{
				for (auto &shape : section->m_cells[cellindex].m_shapestranslucent)
				{
					auto const radius{shape.radius()};
					auto const &shapedata{shape.data()};
					put(data, static_cast<std::uint32_t>(shapedata.material - 1));
					put(data, static_cast<std::uint32_t>(cellindex));
					put(data, shapedata.rangesquared_min);
					put(data, shapedata.rangesquared_max);
					for (auto const &colour : {shapedata.lighting.diffuse, shapedata.lighting.ambient, shapedata.lighting.specular})
					{
						put(data, colour.r);
						put(data, colour.g);
						put(data, colour.b);
						put(data, colour.a);
					}
					put(data, shapedata.area.center.x);
					put(data, shapedata.area.center.y);
					put(data, shapedata.area.center.z);
					put(data, radius);
					put(data, static_cast<std::uint32_t>(shapedata.vertices.size()));
					for (auto const &vertex : shapedata.vertices)
					{
						glm::vec3 const position{vertex.position - section->m_area.center};
						put(data, position.x);
						put(data, position.y);
						put(data, position.z);
						put(data, vertex.normal.x);
						put(data, vertex.normal.y);
						put(data, vertex.normal.z);
						put(data, vertex.texture.x);
						put(data, vertex.texture.y);
					}
					++shapecount;
					vertexcount += shapedata.vertices.size();
				}
			}
			if (data.size() > std::numeric_limits<std::uint32_t>::max())
			{
				Message = "too much geometry in a single section of the scene";
				output.close();
				std::error_code error;
				std::filesystem::remove(temporaryfile, error);
				return false;
			}
			output.write(data.data(), static_cast<std::streamsize>(data.size()));
			put(directory, sectionindex);
			put(directory, offset);
			put(directory, static_cast<std::uint32_t>(data.size()));
			put(directory, section->m_area.radius);
			offset += data.size();
		}
		// fill in the parts left for later
		output.seekp(static_cast<std::streamoff>(directoryposition));
		output.write(directory.data(), static_cast<std::streamsize>(directory.size()));
		data.clear();
		put(data, vertexcount);
		output.seekp(static_cast<std::streamoff>(vertexcountposition));
		output.write(data.data(), static_cast<std::streamsize>(data.size()));
		output.flush();
		if (output.fail())
		{
			Message = "can't write to file \"" + temporaryfile + "\"";
			output.close();
			std::error_code error;
			std::filesystem::remove(temporaryfile, error);
			return false;
		}
	}
	// the finished file replaces the previous version in one go, so an interrupted conversion doesn't leave a broken one behind
	std::error_code error;
	std::filesystem::rename(temporaryfile, Binaryfile, error);
	if (error)
	{
		Message = "can't write to file \"" + Binaryfile + "\"";
		std::filesystem::remove(temporaryfile, error);
		return false;
	}

	Message = std::to_string(sections.size()) + " section(s), " + std::to_string(shapecount) + " shape(s), " + std::to_string(vertexcount / 3) + " triangle(s)";
	return true;
}

// converts specified text terrain file to binary format. returns: true on success, false otherwise
bool terrain_file::convert(std::string const &Textfile, std::string const &Binaryfile, std::string *Message)
{
	std::string message;
	std::string text;
	auto result{false};
	if (read_file(Textfile, text))
	{
		result = write(text, Binaryfile, message);
	}
	else
	{
		message = "can't read the file";
	}
	if (Message != nullptr)
	{
		*Message = message;
	}
	return result;
}

// makes sure binary version of specified text terrain file is in place and matches the text
terrain_file::state terrain_file::prepare(std::string const &Textfile, std::string const &Binaryfile)
{
	file_header header;
	auto const binarypresent{read_header(Binaryfile, header)};

	std::error_code error;
	auto const textsize{std::filesystem::file_size(Textfile, error)};
	if (error)
	{
		// with no text to check the binary file against it's taken as it is
		return (binarypresent ? state::binary : state::missing);
	}

	std::string text;
	auto textloaded{false};
	if (binarypresent && (header.sourcesize == textsize))
	{
		// the length is the same; the checksum tells whether the content is, too
		textloaded = read_file(Textfile, text);
		if (textloaded && (text.size() == header.sourcesize) && (checksum(text.data(), text.size()) == header.sourcechecksum))
		{
			return state::binary;
		}
	}
	if (false == textloaded && false == read_file(Textfile, text))
	{
		ErrorLog("Bad file: can't read terrain file \"" + Textfile + "\"");
		// NOTE: binary file which doesn't match the text isn't used, the scenery is expected to show what its files say
		return state::missing;
	}

	WriteLog("Terrain file \"" + Textfile + "\": " + (binarypresent ? "binary version doesn't match the text" : "no usable binary version found") + ", converting...");
	std::string message;
	if (write(text, Binaryfile, message))
	{
		WriteLog("Terrain file \"" + Textfile + "\": conversion done, " + message);
		return state::binary;
	}
	ErrorLog("Bad file: terrain file \"" + Textfile + "\" wasn't converted to binary format, " + message + ". The file will be loaded as text");
	if (binarypresent)
	{
		// the binary file left behind is of no use, and would be taken for the real thing if the text went missing
		std::filesystem::remove(Binaryfile, error);
	}
	return state::text;
}

// makes content of specified binary terrain file a part of provided region
bool terrain_file::attach(std::string const &Binaryfile, basic_region &Region, bool Deferred)
{
	std::ifstream input{Binaryfile, std::ios::binary};
	file_header header;
	if (input.fail() || false == read_header(input, header))
	{
		ErrorLog("Bad file: \"" + Binaryfile + "\" is of either unrecognized type or version");
		return false;
	}
	auto source{std::make_shared<terrain_source>()};
	source->file = Binaryfile;
	source->materials.reserve(header.materialcount);
	for (std::uint32_t index{0}; index < header.materialcount; ++index)
	{
		std::string name;
		if (false == static_cast<bool>(std::getline(input, name, '\0')))
		{
			break;
		}
		source->materials.emplace_back(fetch_material(name));
	}

	std::string data(header.sectioncount * (sizeof(std::uint32_t) + sizeof(std::uint64_t) + sizeof(std::uint32_t) + sizeof(float)), '\0');
	input.read(data.data(), static_cast<std::streamsize>(data.size()));
	data_reader reader{data.data(), static_cast<std::size_t>(std::max<std::streamsize>(0, input.gcount()))};

	std::uint32_t sectioncount{0};
	for (std::uint32_t index{0}; index < header.sectioncount && reader.good(); ++index)
	{
		auto const sectionindex{reader.get<std::uint32_t>()};
		terrain_block block;
		block.source = source;
		block.offset = reader.get<std::uint64_t>();
		block.size = reader.get<std::uint32_t>();
		auto const radius{reader.get<float>()};
		if (false == reader.good() || sectionindex >= Region.m_sections.size())
		{
			break;
		}
		// location of the section follows from its index, see basic_region::section()
		auto const column{static_cast<int>(sectionindex % EU07_REGIONSIDESECTIONCOUNT)};
		auto const row{static_cast<int>(sectionindex / EU07_REGIONSIDESECTIONCOUNT)};
		auto const centeroffset{-(EU07_REGIONSIDESECTIONCOUNT / 2 * EU07_SECTIONSIZE) + EU07_SECTIONSIZE / 2};
		auto &section{Region.section(glm::dvec3{static_cast<double>(centeroffset + column * EU07_SECTIONSIZE), 0.0, static_cast<double>(centeroffset + row * EU07_SECTIONSIZE)})};
		// the section has to be seen as reaching as far as its geometry does, before that geometry is loaded
		section.m_area.radius = std::max(section.m_area.radius, radius);
		section.m_terrain.emplace_back(block);
		if (false == Deferred)
		{
			section.load_terrain();
		}
		++sectioncount;
	}
	if (false == reader.good() || sectioncount != header.sectioncount || source->materials.size() != header.materialcount)
	{
		ErrorLog("Bad file: \"" + Binaryfile + "\" is damaged, part of its content may be missing");
	}
	WriteLog("Terrain file \"" + Binaryfile + "\": " + std::to_string(sectioncount) + " section(s), " + std::to_string(header.vertexcount / 3) + " triangle(s)" +
	         (Deferred ? ", loaded when needed" : ""));
	return true;
}

// loads specified piece of a binary terrain file into provided section
void terrain_file::load(basic_section &Section, terrain_block const &Block)
{
	if (Block.source == nullptr)
	{
		return;
	}
	auto const &source{*Block.source};

	std::string data(Block.size, '\0');
	{
		std::ifstream input{source.file, std::ios::binary};
		input.seekg(static_cast<std::streamoff>(Block.offset));
		input.read(data.data(), static_cast<std::streamsize>(data.size()));
		if (input.fail() || static_cast<std::size_t>(std::max<std::streamsize>(0, input.gcount())) != data.size())
		{
			ErrorLog("Bad file: failed to load terrain section from \"" + source.file + "\"");
			return;
		}
	}
	data_reader reader{data.data(), data.size()};
	if (reader.get<std::uint32_t>() != EU07_TERRAINBLOCK)
	{
		// the file isn't what it was when its directory was read
		ErrorLog("Bad file: failed to load terrain section from \"" + source.file + "\", the file was changed");
		return;
	}
	reader.get<std::uint32_t>(); // section index
	auto shapecount{reader.get<std::uint32_t>()};

	// large pieces of opaque geometry are held by the section itself, merged by their appearance. see basic_section::insert()
	std::vector<shape_node> sectionshapes;
	auto const place = [&](shape_node &Shape, std::vector<shape_node> &Shapes, glm::dvec3 const &Origin) {
		Shape.origin(Origin);
		if (true == Section.m_geometrycreated)
		{
			// the section was put to use already, it won't prepare the geometry on its own
			Shape.create_geometry(Section.m_geometrybank);
		}
		Shapes.emplace_back(std::move(Shape));
	};

	while (shapecount-- > 0 && reader.good())
	{
		auto const materialindex{reader.get<std::uint32_t>()};
		auto const cellindex{reader.get<std::uint32_t>()};
		shape_node shape;
		auto &shapedata{shape.m_data};
		shapedata.rangesquared_min = reader.get<double>();
		shapedata.rangesquared_max = reader.get<double>();
		for (auto *colour : {&shapedata.lighting.diffuse, &shapedata.lighting.ambient, &shapedata.lighting.specular})
		{
			colour->r = reader.get<float>();
			colour->g = reader.get<float>();
			colour->b = reader.get<float>();
			colour->a = reader.get<float>();
		}
		shapedata.area.center.x = reader.get<double>();
		shapedata.area.center.y = reader.get<double>();
		shapedata.area.center.z = reader.get<double>();
		shapedata.area.radius = reader.get<float>();
		auto const vertexcount{reader.get<std::uint32_t>()};
		if (false == reader.good() || false == reader.has(static_cast<std::size_t>(vertexcount) * EU07_TERRAINVERTEXSIZE) || materialindex >= source.materials.size() ||
		    cellindex >= Section.m_cells.size())
		{
			ErrorLog("Bad file: terrain section data in \"" + source.file + "\" is damaged");
			break;
		}
		auto const &material{source.materials[materialindex]};
		if (true == material.skip)
		{
			reader.skip(static_cast<std::size_t>(vertexcount) * EU07_TERRAINVERTEXSIZE);
			continue;
		}
		shapedata.material = material.handle;
		shapedata.translucent = material.translucent;
		shapedata.vertices.resize(vertexcount);
		for (auto &vertex : shapedata.vertices)
		{
			float values[8];
			reader.get(values, 8);
			vertex.position = Section.m_area.center + glm::dvec3{values[0], values[1], values[2]};
			vertex.normal = glm::vec3{values[3], values[4], values[5]};
			vertex.texture = glm::vec2{values[6], values[7]};
			// clamp texture coordinates if texture wrapping is off
			// NOTE: for the text this is done before a large triangle gets divided along the section borders, here it comes after.
			// the coordinates of the vertices added by the division can thus differ, by the clamp margin for a texture mapped the way clamping expects
			if (true == material.clamps)
			{
				vertex.texture.s = std::clamp(vertex.texture.s, 0.001f, 0.999f);
			}
			if (true == material.clampt)
			{
				vertex.texture.t = std::clamp(vertex.texture.t, 0.001f, 0.999f);
			}
		}

		if (true == shapedata.translucent || shapedata.rangesquared_max <= 90000.0 || shapedata.rangesquared_min > 0.0)
		{
			// small, translucent or not always visible shapes are placed in the sub-cells
			auto &cell{Section.m_cells[cellindex]};
			cell.m_active = true;
			cell.m_area.radius = std::max<float>(cell.m_area.radius, static_cast<float>(glm::length(cell.m_area.center - shapedata.area.center) + shapedata.area.radius));
			place(shape, (shapedata.translucent ? cell.m_shapestranslucent : cell.m_shapesopaque), cell.m_area.center);
		}
		else
		{
			auto merged{false};
			for (auto &target : sectionshapes)
			{
				if (true == target.merge(shape))
				{
					merged = true;
					break;
				}
			}
			if (false == merged)
			{
				sectionshapes.emplace_back(std::move(shape));
			}
		}
	}
	for (auto &shape : sectionshapes)
	{
		place(shape, Section.m_shapes, Section.m_area.center);
	}
}

} // namespace scene

//---------------------------------------------------------------------------
