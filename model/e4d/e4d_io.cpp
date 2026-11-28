/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "model/e4d/e4d_io.h"
#include "model/e4d/e4d_dump.h"

#include "meshoptimizer.h"
#include "zstd.h"

#include <format>
#include <iterator>
#include <limits>
#include <thread>
#include <type_traits>
#include <span>

namespace e4d {

std::uint32_t model::intern(std::string_view const Text) {
	if (m_stringids.size() != strings.size()) {
		// the strings were filled from outside (the reader), the lookup is made anew
		m_stringids.clear();
		for (std::uint32_t idx = 0; idx < strings.size(); ++idx)
			m_stringids.try_emplace(strings[idx], idx);
	}
	if (auto const lookup{m_stringids.find(std::string{Text})}; lookup != m_stringids.end())
		return lookup->second;
	auto const id{static_cast<std::uint32_t>(strings.size())};
	strings.emplace_back(Text);
	m_stringids.try_emplace(strings.back(), id);
	return id;
}

std::string_view model::string(std::uint32_t const Id) const {
	return Id < strings.size() ? std::string_view{strings[Id]} : std::string_view{};
}

std::uint32_t model::add_component(std::uint32_t const Type, target_kind const Kind, std::uint32_t const Target, std::span<std::byte const> const Data, std::uint16_t const Version) {
	// the payloads start at multiples of 4, so their fields can be read in place
	while (component_data.size() % 4 != 0)
		component_data.push_back(std::byte{0});
	component_record record;
	record.type = Type;
	record.version = Version;
	record.kind = Kind;
	record.target = Target;
	record.data_offset = static_cast<std::uint32_t>(component_data.size());
	record.data_size = static_cast<std::uint32_t>(Data.size());
	component_data.insert(component_data.end(), Data.begin(), Data.end());
	components.push_back(record);
	return static_cast<std::uint32_t>(components.size() - 1);
}

std::span<std::byte const> model::component_payload(std::uint32_t const Component) const {
	if (Component >= components.size())
		return {};
	auto const &record{components[Component]};
	if (std::uint64_t{record.data_offset} + record.data_size > component_data.size())
		return {};
	return std::span<std::byte const>{component_data}.subspan(record.data_offset, record.data_size);
}

std::uint64_t hash_bytes(std::span<std::byte const> const Data, std::uint64_t const Seed) {
	auto hash{Seed};
	for (auto const byte : Data) {
		hash ^= std::to_integer<std::uint64_t>(byte);
		hash *= 0x100000001b3ull;
	}
	return hash;
}

std::uint64_t hash_name(std::string_view const Name) {
	std::uint64_t hash{0xcbf29ce484222325ull};
	for (auto const character : Name) {
		auto byte{static_cast<unsigned char>(character)};
		if (byte >= 'A' && byte <= 'Z')
			byte = static_cast<unsigned char>(byte - 'A' + 'a');
		hash ^= byte;
		hash *= 0x100000001b3ull;
	}
	return hash;
}

namespace {

// ---------------------------------------------------------------------------------------------------------------------
// writing

template <typename Type_> void append_bytes(std::vector<std::byte> &Output, Type_ const &Value) {
	auto const offset{Output.size()};
	Output.resize(offset + sizeof(Type_));
	std::memcpy(Output.data() + offset, &Value, sizeof(Type_));
}

template <typename Type_> void append_array(std::vector<std::byte> &Output, std::vector<Type_> const &Values) {
	if (Values.empty())
		return;
	auto const offset{Output.size()};
	Output.resize(offset + Values.size() * sizeof(Type_));
	std::memcpy(Output.data() + offset, Values.data(), Values.size() * sizeof(Type_));
}

// which codec of meshoptimizer a stream may go through
enum class stream_codec {
	none,
	vertices,
	indices
};

struct pending_section {
	std::uint32_t id{0};
	std::uint32_t flags{0};
	std::size_t alignment{section_alignment};
	std::vector<std::byte> data;
	stream_codec codec{stream_codec::none};
	std::size_t element{0}; // the size of an element of the stream
};

// the sections smaller than this stay as they are, unpacking them costs more than reading them
constexpr std::size_t packing_threshold{512};

// the vertex codec of meshoptimizer, version 1, level 2
constexpr int vertex_codec_version{1};
constexpr int vertex_codec_level{2};

template <typename Index_> std::size_t encode_indices(std::vector<std::byte> const &Data, std::vector<std::byte> &Encoded) {
	std::vector<Index_> indices(Data.size() / sizeof(Index_));
	std::memcpy(indices.data(), Data.data(), indices.size() * sizeof(Index_));
	auto const last{std::ranges::max(indices)};
	Encoded.resize(meshopt_encodeIndexSequenceBound(indices.size(), std::size_t{last} + 1));
	return meshopt_encodeIndexSequence(reinterpret_cast<unsigned char *>(Encoded.data()), Encoded.size(), indices.data(), indices.size());
}

// the stream in its codec. false when it has none, is too small, or the codec doesn't make it smaller
bool encode_stream(pending_section const &Section, std::vector<std::byte> &Encoded) {
	if (Section.codec == stream_codec::none || Section.data.size() < packing_threshold || Section.data.size() % Section.element != 0)
		return false;
	auto const count{Section.data.size() / Section.element};
	std::size_t size{0};
	if (Section.codec == stream_codec::vertices) {
		Encoded.resize(meshopt_encodeVertexBufferBound(count, Section.element));
		size = meshopt_encodeVertexBufferLevel(reinterpret_cast<unsigned char *>(Encoded.data()), Encoded.size(), Section.data.data(), count, Section.element, vertex_codec_level,
		    vertex_codec_version);
	}
	else if (Section.element == sizeof(std::uint16_t))
		size = encode_indices<std::uint16_t>(Section.data, Encoded);
	else
		size = encode_indices<std::uint32_t>(Section.data, Encoded);
	if (size == 0 || size >= Section.data.size())
		return false;
	Encoded.resize(size);
	return true;
}

class file_builder {
  public:
	template <typename Type_> void table(std::uint32_t const Id, std::vector<Type_> const &Records, bool const Required) {
		if (Records.empty() && false == Required)
			return;
		std::vector<std::byte> data;
		append_bytes(data, table_header{static_cast<std::uint32_t>(Records.size()), static_cast<std::uint32_t>(sizeof(Type_))});
		append_array(data, Records);
		add(Id, Required, section_alignment, std::move(data));
	}
	template <typename Type_> void stream(std::uint32_t const Id, std::vector<Type_> const &Values, bool const Required, stream_codec const Codec = stream_codec::none) {
		if (Values.empty() && false == Required)
			return;
		std::vector<std::byte> data;
		append_array(data, Values);
		add(Id, Required, gpu_alignment, std::move(data));
		m_sections.back().codec = Codec;
		m_sections.back().element = sizeof(Type_);
	}
	void raw(std::uint32_t const Id, std::uint32_t const Flags, std::vector<std::byte> Data) {
		m_sections.push_back({Id, Flags & section_flags::required, section_alignment, std::move(Data)});
	}
	void add(std::uint32_t const Id, bool const Required, std::size_t const Alignment, std::vector<std::byte> Data) {
		m_sections.push_back({Id, Required ? section_flags::required : 0u, Alignment, std::move(Data)});
	}
	std::vector<std::byte> finish(model const &Model, write_options const &Options);

  private:
	static void pad_to(std::vector<std::byte> &Output, std::size_t const Alignment) {
		while (Output.size() % Alignment != 0)
			Output.push_back(std::byte{0});
	}
	static bool compress(std::span<std::byte const> Data, std::vector<std::byte> &Packed, int Level);

	std::vector<pending_section> m_sections;
};

bool file_builder::compress(std::span<std::byte const> const Data, std::vector<std::byte> &Packed, int const Level) {
	if (Data.size() < packing_threshold)
		return false;
	Packed.resize(ZSTD_compressBound(Data.size()));
	auto const size{ZSTD_compress(Packed.data(), Packed.size(), Data.data(), Data.size(), Level)};
	if (ZSTD_isError(size) || size > Data.size() - Data.size() / 8)
		return false;
	Packed.resize(size);
	return true;
}

std::vector<std::byte> file_builder::finish(model const &Model, write_options const &Options) {
	std::vector<std::byte> output(sizeof(file_header) + m_sections.size() * sizeof(section_entry));
	std::vector<section_entry> entries;
	entries.reserve(m_sections.size());
	std::vector<std::byte> packed;
	std::vector<std::byte> encoded;
	for (auto const &section : m_sections) {
		pad_to(output, section.alignment);
		section_entry entry;
		entry.id = section.id;
		entry.flags = section.flags;
		entry.offset = output.size();
		entry.size = section.data.size();
		std::span<std::byte const> bytes{section.data};
		auto const coded{Options.codec && encode_stream(section, encoded)};
		if (coded) {
			entry.flags |= section_flags::meshopt;
			bytes = encoded;
		}
		if (Options.compress && (false == coded || Options.compress_streams) && compress(bytes, packed, Options.level)) {
			entry.flags |= section_flags::zstd;
			bytes = packed;
		}
		output.insert(output.end(), bytes.begin(), bytes.end());
		entry.stored_size = output.size() - entry.offset;
		entries.push_back(entry);
	}
	std::memcpy(output.data() + sizeof(file_header), entries.data(), entries.size() * sizeof(section_entry));

	file_header header;
	header.flags = Model.flags;
	header.section_count = static_cast<std::uint32_t>(entries.size());
	header.toc_offset = sizeof(file_header);
	header.file_size = output.size();
	header.source_hash = Model.source_hash;
	header.content_hash = hash_bytes(std::span<std::byte const>{output}.subspan(sizeof(file_header)));
	std::memcpy(output.data(), &header, sizeof(file_header));
	return output;
}

std::vector<std::byte> encode_strings(std::vector<std::string> const &Strings) {
	std::vector<std::uint32_t> offsets;
	offsets.reserve(Strings.size() + 1);
	std::uint32_t offset{0};
	for (auto const &text : Strings) {
		offsets.push_back(offset);
		offset += static_cast<std::uint32_t>(text.size());
	}
	offsets.push_back(offset);
	std::vector<std::byte> data;
	append_bytes(data, static_cast<std::uint32_t>(Strings.size()));
	append_bytes(data, offset);
	append_array(data, offsets);
	for (auto const &text : Strings) {
		auto const *bytes{reinterpret_cast<std::byte const *>(text.data())};
		data.insert(data.end(), bytes, bytes + text.size());
	}
	return data;
}

// ---------------------------------------------------------------------------------------------------------------------
// reading

// a section of the file, unpacked
class section_bytes {
  public:
	section_bytes(std::uint32_t const Id, std::span<std::byte const> const Bytes) : m_id(Id), m_bytes(Bytes) {}

	template <typename Type_> bool table(std::vector<Type_> &Output, std::string &Error) const {
		table_header header;
		if (m_bytes.size() < sizeof(header))
			return fail(Error, "is shorter than its table header");
		std::memcpy(&header, m_bytes.data(), sizeof(header));
		if (header.stride < sizeof(Type_))
			return fail(Error, std::format("has records of {} bytes, {} expected at least", header.stride, sizeof(Type_)));
		if (std::uint64_t{header.count} * header.stride > m_bytes.size() - sizeof(header))
			return fail(Error, std::format("holds {} records of {} bytes in {} bytes", header.count, header.stride, m_bytes.size() - sizeof(header)));
		Output.resize(header.count);
		for (std::size_t idx = 0; idx < header.count; ++idx)
			std::memcpy(&Output[idx], m_bytes.data() + sizeof(header) + idx * header.stride, sizeof(Type_));
		return true;
	}

	template <typename Type_> bool stream(std::vector<Type_> &Output, std::string &Error) const {
		if (m_bytes.size() % sizeof(Type_) != 0)
			return fail(Error, std::format("of {} bytes isn't a whole number of elements of {} bytes", m_bytes.size(), sizeof(Type_)));
		Output.resize(m_bytes.size() / sizeof(Type_));
		if (false == Output.empty())
			std::memcpy(Output.data(), m_bytes.data(), m_bytes.size());
		return true;
	}

	bool strings(std::vector<std::string> &Output, std::string &Error) const;

	std::span<std::byte const> bytes() const {
		return m_bytes;
	}

  private:
	bool fail(std::string &Error, std::string_view const What) const {
		Error = std::format("section {} {}", fourcc_name(m_id), What);
		return false;
	}

	std::uint32_t m_id;
	std::span<std::byte const> m_bytes;
};

bool section_bytes::strings(std::vector<std::string> &Output, std::string &Error) const {
	std::array<std::uint32_t, 2> head{};
	if (m_bytes.size() < sizeof(head))
		return fail(Error, "is shorter than its head");
	std::memcpy(head.data(), m_bytes.data(), sizeof(head));
	auto const count{std::uint64_t{head[0]}};
	auto const offsetsend{sizeof(head) + (count + 1) * sizeof(std::uint32_t)};
	if (offsetsend > m_bytes.size() || offsetsend + head[1] > m_bytes.size())
		return fail(Error, std::format("can't hold {} strings of {} bytes", count, head[1]));
	std::vector<std::uint32_t> offsets(count + 1);
	std::memcpy(offsets.data(), m_bytes.data() + sizeof(head), offsets.size() * sizeof(std::uint32_t));
	if (offsets.front() != 0 || offsets.back() != head[1] || false == std::ranges::is_sorted(offsets))
		return fail(Error, "has the bounds of its strings out of order");
	auto const *blob{reinterpret_cast<char const *>(m_bytes.data() + offsetsend)};
	Output.clear();
	Output.reserve(count);
	for (std::size_t idx = 0; idx < count; ++idx)
		Output.emplace_back(blob + offsets[idx], offsets[idx + 1] - offsets[idx]);
	return true;
}

// the largest section unpacked, so a broken file can't make the reader take all the memory
constexpr std::uint64_t section_size_limit{std::uint64_t{1} << 30};

class file_reader {
  public:
	file_reader(std::span<std::byte const> const File, read_options const &Options) : m_file(File), m_options(Options) {}
	read_result run(model &Model);

  private:
	bool read_header(file_header &Header);
	bool read_entries(file_header const &Header, std::vector<section_entry> &Entries);
	bool unpack(section_entry const &Entry, std::vector<std::byte> &Storage, std::span<std::byte const> &Bytes);
	bool dispatch(section_entry const &Entry, section_bytes const &Section, model &Model);
	bool vertex_stream(section_entry const &Entry, model &Model, bool &Handled);
	bool read_streams(std::vector<section_entry> const &Sections, model &Model);
	template <typename Type_> bool unpack_into(section_entry const &Entry, std::vector<Type_> &Output, stream_codec Codec);
	bool unzstd(section_entry const &Entry, std::size_t Limit, std::vector<std::byte> &Output);
	bool decode(section_entry const &Entry, std::span<std::byte const> Encoded, void *Output, std::size_t Element, stream_codec Codec);
	bool fail(std::string Error) {
		m_error = std::move(Error);
		return false;
	}

	std::span<std::byte const> m_file;
	read_options m_options;
	std::string m_error;
};

bool file_reader::read_header(file_header &Header) {
	if (m_file.size() < sizeof(file_header))
		return fail("the file is shorter than the header");
	std::memcpy(&Header, m_file.data(), sizeof(file_header));
	if (Header.magic != file_header{}.magic)
		return fail("the file isn't an E4D model");
	if (Header.major != version_major)
		return fail(std::format("the model has version {}.{}, this program reads version {}", Header.major, Header.minor, version_major));
	if (Header.file_size > m_file.size())
		return fail(std::format("the file is cut short: {} bytes of {}", m_file.size(), Header.file_size));
	m_file = m_file.first(Header.file_size);
	if (m_options.verify_hash && hash_bytes(m_file.subspan(sizeof(file_header))) != Header.content_hash)
		return fail("the content of the file doesn't match its hash");
	return true;
}

bool file_reader::read_entries(file_header const &Header, std::vector<section_entry> &Entries) {
	auto const tablesize{std::uint64_t{Header.section_count} * sizeof(section_entry)};
	if (Header.toc_offset > m_file.size() || tablesize > m_file.size() - Header.toc_offset)
		return fail("the list of the sections lies outside the file");
	Entries.resize(Header.section_count);
	if (false == Entries.empty())
		std::memcpy(Entries.data(), m_file.data() + Header.toc_offset, tablesize);
	std::vector<std::uint32_t> seen;
	for (auto const &entry : Entries) {
		if (entry.offset > m_file.size() || entry.stored_size > m_file.size() - entry.offset)
			return fail(std::format("section {} lies outside the file", fourcc_name(entry.id)));
		if (entry.size > section_size_limit)
			return fail(std::format("section {} claims {} bytes", fourcc_name(entry.id), entry.size));
		if (std::ranges::find(seen, entry.id) != seen.end())
			return fail(std::format("section {} appears twice", fourcc_name(entry.id)));
		seen.push_back(entry.id);
	}
	return true;
}

bool file_reader::unpack(section_entry const &Entry, std::vector<std::byte> &Storage, std::span<std::byte const> &Bytes) {
	auto const stored{m_file.subspan(Entry.offset, Entry.stored_size)};
	if ((Entry.flags & section_flags::meshopt) != 0)
		return fail(std::format("section {} is in a codec only the vertex and index streams may be in", fourcc_name(Entry.id)));
	if ((Entry.flags & section_flags::zstd) == 0) {
		if (Entry.stored_size != Entry.size)
			return fail(std::format("section {} has {} bytes stored and {} declared", fourcc_name(Entry.id), Entry.stored_size, Entry.size));
		Bytes = stored;
		return true;
	}
	// the size the frame declares has to be the one of the section before any memory is taken for it
	if (ZSTD_getFrameContentSize(stored.data(), stored.size()) != Entry.size)
		return fail(std::format("section {} doesn't unpack to its {} bytes", fourcc_name(Entry.id), Entry.size));
	Storage.resize(Entry.size);
	auto const size{ZSTD_decompress(Storage.data(), Storage.size(), stored.data(), stored.size())};
	if (ZSTD_isError(size) || size != Entry.size)
		return fail(std::format("section {} doesn't unpack to its {} bytes", fourcc_name(Entry.id), Entry.size));
	Bytes = Storage;
	return true;
}

// the zstd frame of the section, of at most Limit bytes
bool file_reader::unzstd(section_entry const &Entry, std::size_t const Limit, std::vector<std::byte> &Output) {
	auto const stored{m_file.subspan(Entry.offset, Entry.stored_size)};
	auto const size{ZSTD_getFrameContentSize(stored.data(), stored.size())};
	if (size == ZSTD_CONTENTSIZE_UNKNOWN || size == ZSTD_CONTENTSIZE_ERROR || size > Limit)
		return fail(std::format("section {} doesn't unpack to a size its codec can have", fourcc_name(Entry.id)));
	Output.resize(static_cast<std::size_t>(size));
	auto const unpacked{ZSTD_decompress(Output.data(), Output.size(), stored.data(), stored.size())};
	if (ZSTD_isError(unpacked) || unpacked != size)
		return fail(std::format("section {} doesn't unpack to its {} bytes", fourcc_name(Entry.id), size));
	return true;
}

bool file_reader::decode(section_entry const &Entry, std::span<std::byte const> const Encoded, void *const Output, std::size_t const Element, stream_codec const Codec) {
	auto const count{static_cast<std::size_t>(Entry.size) / Element};
	auto const *data{reinterpret_cast<unsigned char const *>(Encoded.data())};
	auto const result{Codec == stream_codec::vertices ? meshopt_decodeVertexBuffer(Output, count, Element, data, Encoded.size())
	                                                  : meshopt_decodeIndexSequence(Output, count, Element, data, Encoded.size())};
	if (result != 0)
		return fail(std::format("section {} doesn't decode to its {} bytes", fourcc_name(Entry.id), Entry.size));
	return true;
}

// a stream unpacked straight into its vector, without a copy in between: the vertices and the indices are most of the file
template <typename Type_> bool file_reader::unpack_into(section_entry const &Entry, std::vector<Type_> &Output, stream_codec const Codec) {
	if (Entry.size % sizeof(Type_) != 0)
		return fail(std::format("section {} of {} bytes isn't a whole number of elements of {} bytes", fourcc_name(Entry.id), Entry.size, sizeof(Type_)));
	auto const stored{m_file.subspan(Entry.offset, Entry.stored_size)};
	if ((Entry.flags & section_flags::meshopt) != 0) {
		auto const count{static_cast<std::size_t>(Entry.size) / sizeof(Type_)};
		// the result of the codec is never larger than its bound, which caps what a broken zstd frame can claim
		std::size_t bound{0};
		if constexpr (std::is_integral_v<Type_>)
			bound = meshopt_encodeIndexSequenceBound(count, std::size_t{std::numeric_limits<Type_>::max()});
		else
			bound = meshopt_encodeVertexBufferBound(count, sizeof(Type_));
		std::vector<std::byte> unpacked;
		if ((Entry.flags & section_flags::zstd) != 0 && false == unzstd(Entry, bound, unpacked))
			return false;
		auto const encoded{(Entry.flags & section_flags::zstd) != 0 ? std::span<std::byte const>{unpacked} : stored};
		Output.resize(count);
		return decode(Entry, encoded, Output.data(), sizeof(Type_), Codec);
	}
	if ((Entry.flags & section_flags::zstd) == 0) {
		if (Entry.stored_size != Entry.size)
			return fail(std::format("section {} has {} bytes stored and {} declared", fourcc_name(Entry.id), Entry.stored_size, Entry.size));
		Output.resize(Entry.size / sizeof(Type_));
		if (false == Output.empty())
			std::memcpy(Output.data(), stored.data(), Entry.size);
		return true;
	}
	if (ZSTD_getFrameContentSize(stored.data(), stored.size()) != Entry.size)
		return fail(std::format("section {} doesn't unpack to its {} bytes", fourcc_name(Entry.id), Entry.size));
	Output.resize(Entry.size / sizeof(Type_));
	auto const size{ZSTD_decompress(Output.data(), Entry.size, stored.data(), stored.size())};
	if (ZSTD_isError(size) || size != Entry.size)
		return fail(std::format("section {} doesn't unpack to its {} bytes", fourcc_name(Entry.id), Entry.size));
	return true;
}

bool file_reader::vertex_stream(section_entry const &Entry, model &Model, bool &Handled) {
	using enum stream_codec;
	Handled = true;
	switch (Entry.id) {
		case section::positions: return unpack_into(Entry, Model.positions, vertices);
		case section::attributes: return unpack_into(Entry, Model.vertex_attributes, vertices);
		case section::skin_weights: return unpack_into(Entry, Model.weights, vertices);
		case section::skin_weights_wide: return unpack_into(Entry, Model.weights_wide, vertices);
		case section::userdata: return unpack_into(Entry, Model.vertex_userdata, vertices);
		case section::bindings: return unpack_into(Entry, Model.bindings, vertices);
		case section::indices16: return unpack_into(Entry, Model.indices16, indices);
		case section::indices32: return unpack_into(Entry, Model.indices32, indices);
		default: Handled = false; return true;
	}
}

bool is_vertex_stream(std::uint32_t const Id) {
	switch (Id) {
		case section::positions:
		case section::attributes:
		case section::skin_weights:
		case section::skin_weights_wide:
		case section::userdata:
		case section::bindings:
		case section::indices16:
		case section::indices32: return true;
		default: return false;
	}
}

// a stream of this size or more unpacks on a thread of its own, beside the others
constexpr std::uint64_t parallel_stream_size{std::uint64_t{1} << 20};

// the vertex and index streams, the big ones at once: each goes into a vector of its own, and the errors are kept apart
// and told in the order of the sections
bool file_reader::read_streams(std::vector<section_entry> const &Sections, model &Model) {
	std::vector<section_entry> streams;
	std::ranges::copy_if(Sections, std::back_inserter(streams), [](section_entry const &Entry) { return is_vertex_stream(Entry.id); });
	std::vector<std::string> errors(streams.size());
	auto const unpack_one{[this, &streams, &errors, &Model](std::size_t const Index) {
		file_reader worker{m_file, m_options};
		bool handled{false};
		if (false == worker.vertex_stream(streams[Index], Model, handled))
			errors[Index] = std::move(worker.m_error);
	}};
	{
		std::vector<std::jthread> workers;
		for (std::size_t idx = 0; idx < streams.size(); ++idx) {
			if (streams[idx].size >= parallel_stream_size)
				workers.emplace_back(unpack_one, idx);
			else
				unpack_one(idx);
		}
	} // the workers join here
	if (auto const failed{std::ranges::find_if(errors, [](std::string const &Error) { return false == Error.empty(); })}; failed != errors.end())
		return fail(*failed);
	return true;
}

bool file_reader::dispatch(section_entry const &Entry, section_bytes const &Section, model &Model) {
	std::string error;
	bool known{true};
	bool result{true};
	switch (Entry.id) {
		case section::strings: result = Section.strings(Model.strings, error); break;
		case section::name_index: result = Section.table(Model.name_index, error); break;
		case section::nodes: result = Section.table(Model.nodes, error); break;
		case section::transforms: result = Section.table(Model.transforms, error); break;
		case section::meshes: result = Section.table(Model.meshes, error); break;
		case section::primitives: result = Section.table(Model.primitives, error); break;
		case section::positions: result = Section.stream(Model.positions, error); break;
		case section::attributes: result = Section.stream(Model.vertex_attributes, error); break;
		case section::skin_weights: result = Section.stream(Model.weights, error); break;
		case section::skin_weights_wide: result = Section.stream(Model.weights_wide, error); break;
		case section::userdata: result = Section.stream(Model.vertex_userdata, error); break;
		case section::bindings: result = Section.stream(Model.bindings, error); break;
		case section::indices16: result = Section.stream(Model.indices16, error); break;
		case section::indices32: result = Section.stream(Model.indices32, error); break;
		case section::materials: result = Section.table(Model.materials, error); break;
		case section::skins: result = Section.table(Model.skins, error); break;
		case section::skin_joints: result = Section.table(Model.skin_joints, error); break;
		case section::vertex_groups: result = Section.table(Model.vertex_groups, error); break;
		case section::vertex_group_items: result = Section.stream(Model.vertex_group_items, error); break;
		case section::vertex_group_weights: result = Section.stream(Model.vertex_group_weights, error); break;
		case section::components: result = Section.table(Model.components, error); break;
		case section::component_data: result = Section.stream(Model.component_data, error); break;
		case section::bounds: result = Section.table(Model.bounds, error); break;
		case section::edit: result = Section.table(Model.edit, error); break;
		default: known = false; break;
	}
	if (false == result)
		return fail(error);
	if (known)
		return true;
	if ((Entry.flags & section_flags::required) != 0)
		return fail(std::format("the model needs section {}, which this program doesn't know", fourcc_name(Entry.id)));
	if (m_options.keep_unknown) {
		auto const bytes{Section.bytes()};
		Model.unknown.push_back({Entry.id, Entry.flags & section_flags::required, std::vector<std::byte>(bytes.begin(), bytes.end())});
	}
	return true;
}

read_result file_reader::run(model &Model) {
	Model = model{};
	file_header header;
	std::vector<section_entry> sections;
	if (false == read_header(header) || false == read_entries(header, sections))
		return {false, m_error};
	Model.flags = header.flags;
	Model.source_hash = header.source_hash;
	std::vector<std::byte> storage;
	if (false == read_streams(sections, Model)) {
		Model = model{};
		return {false, m_error};
	}
	for (auto const &entry : sections) {
		if (is_vertex_stream(entry.id))
			continue;
		std::span<std::byte const> bytes;
		if (false == unpack(entry, storage, bytes) || false == dispatch(entry, section_bytes{entry.id, bytes}, Model)) {
			Model = model{};
			return {false, m_error};
		}
	}
	for (auto const required : {section::strings, section::nodes, section::meshes, section::primitives, section::positions, section::attributes, section::materials}) {
		if (std::ranges::none_of(sections, [required](section_entry const &Entry) { return Entry.id == required; })) {
			Model = model{};
			return {false, std::format("the model has no section {}", fourcc_name(required))};
		}
	}
	if (auto const faults{validate(Model, 1)}; false == faults.empty()) {
		Model = model{};
		return {false, faults.front()};
	}
	return {true, {}};
}

// ---------------------------------------------------------------------------------------------------------------------
// checking

bool valid_string(model const &Model, std::uint32_t const Id) {
	return Id == no_index || Id < Model.strings.size();
}

bool valid_node(model const &Model, std::uint32_t const Id) {
	return Id < Model.nodes.size();
}

bool valid_optional_node(model const &Model, std::uint32_t const Id) {
	return Id == no_index || valid_node(Model, Id);
}

bool valid_optional_component(model const &Model, std::uint32_t const Id) {
	return Id == no_index || Id < Model.components.size();
}

bool all_nodes(model const &Model, payload_reader &Reader, std::size_t const Count) {
	for (std::size_t idx = 0; idx < Count; ++idx) {
		std::uint32_t node{0};
		if (false == Reader.get(node) || false == valid_node(Model, node))
			return false;
	}
	return true;
}

bool valid_properties(model const &Model, payload_reader &Reader) {
	std::uint32_t count{0};
	if (false == Reader.get(count))
		return false;
	for (std::uint32_t idx = 0; idx < count; ++idx) {
		payload::property property;
		if (false == Reader.get(property) || false == valid_string(Model, property.key))
			return false;
		auto const reference{property.value[0]};
		using enum payload::value_type;
		if ((property.type == string && false == valid_string(Model, reference)) || (property.type == node && false == valid_optional_node(Model, reference)) ||
		    (property.type == component && false == valid_optional_component(Model, reference)) || property.type > component)
			return false;
	}
	return true;
}

bool valid_spring(model const &Model, payload_reader &Reader) {
	payload::spring spring;
	return Reader.get(spring) && valid_optional_node(Model, spring.anchor_a) && valid_optional_node(Model, spring.anchor_b) && valid_optional_component(Model, spring.socket_a) &&
	       valid_optional_component(Model, spring.socket_b);
}

bool valid_chain(model const &Model, payload_reader &Reader) {
	payload::chain chain;
	if (false == Reader.get(chain) || false == valid_optional_component(Model, chain.end_a) || false == valid_optional_component(Model, chain.end_b))
		return false;
	if (chain.skin >= static_cast<std::int64_t>(Model.skins.size()))
		return false;
	if (false == all_nodes(Model, Reader, chain.joint_count))
		return false;
	auto const segments{chain.joint_count > 0 ? chain.joint_count - 1u : 0u};
	return Reader.remaining() >= segments * sizeof(float);
}

bool valid_variants(model const &Model, payload_reader &Reader) {
	payload::variants variants;
	if (false == Reader.get(variants) || false == valid_string(Model, variants.name))
		return false;
	if (variants.state_count > 0 && variants.default_state >= variants.state_count)
		return false;
	std::uint64_t listsize{0};
	for (std::uint32_t idx = 0; idx < variants.state_count; ++idx) {
		payload::variant_state state;
		if (false == Reader.get(state) || false == valid_string(Model, state.name))
			return false;
		listsize = std::max(listsize, std::uint64_t{state.node_first} + state.node_count);
	}
	return all_nodes(Model, Reader, listsize);
}

bool valid_channel(model const &Model, payload_reader &Reader) {
	payload::channel channel;
	if (false == Reader.get(channel) || false == valid_string(Model, channel.name))
		return false;
	for (std::uint32_t idx = 0; idx < channel.binding_count; ++idx) {
		payload::channel_binding binding;
		if (false == Reader.get(binding) || binding.kind > payload::binding_kind::light_slot || false == valid_optional_component(Model, binding.target))
			return false;
	}
	return true;
}

bool valid_light_slot(model const &Model, payload_reader &Reader) {
	payload::light_slot slot;
	if (false == Reader.get(slot) || false == all_nodes(Model, Reader, std::size_t{slot.on_count} + slot.off_count))
		return false;
	for (std::uint32_t idx = 0; idx < slot.light_count; ++idx) {
		std::uint32_t light{0};
		if (false == Reader.get(light) || light >= Model.components.size() || Model.components[light].type != component::light)
			return false;
	}
	return true;
}

bool valid_stars(model const &Model, component_record const &Record, payload_reader &Reader) {
	payload::legacy_stars stars;
	if (false == Reader.get(stars) || Reader.remaining() < std::uint64_t{stars.count} * sizeof(std::uint32_t))
		return false;
	// the colours go with the vertices of the mesh of the node
	if (Record.kind != target_kind::node || Record.target >= Model.nodes.size())
		return false;
	auto const mesh{Model.nodes[Record.target].mesh};
	if (mesh < 0 || mesh >= static_cast<std::int64_t>(Model.meshes.size()))
		return false;
	std::uint64_t vertices{0};
	auto const &record{Model.meshes[mesh]};
	for (auto idx = record.primitive_first; idx < record.primitive_first + record.primitive_count && idx < Model.primitives.size(); ++idx)
		vertices += Model.primitives[idx].vertex_count;
	return vertices == stars.count;
}

template <typename Type_> bool valid_fixed(payload_reader &Reader) {
	Type_ value;
	return Reader.get(value);
}

// whether the payload of the component holds what its type says, with references in range. unknown types pass
bool valid_payload(model const &Model, component_record const &Record, std::span<std::byte const> const Data) {
	payload_reader reader{Data};
	switch (Record.type) {
		case component::properties: return valid_properties(Model, reader);
		case component::socket: {
			payload::socket socket;
			return reader.get(socket) && valid_string(Model, socket.role);
		}
		case component::joint: {
			payload::joint joint;
			return reader.get(joint) && joint.kind <= payload::joint_kind::screw && valid_string(Model, joint.role);
		}
		case component::spring: return valid_spring(Model, reader);
		case component::chain: return valid_chain(Model, reader);
		case component::variants: return valid_variants(Model, reader);
		case component::channel: return valid_channel(Model, reader);
		case component::light_slot: return valid_light_slot(Model, reader);
		case component::light: return valid_fixed<payload::light>(reader);
		case component::emitter: return valid_fixed<payload::emitter>(reader);
		case component::model_reference: {
			payload::model_reference reference;
			return reader.get(reference) && valid_string(Model, reference.path) && valid_string(Model, reference.skin) && valid_optional_component(Model, reference.socket);
		}
		case component::collision: {
			payload::collision collision;
			return reader.get(collision) && valid_string(Model, collision.role);
		}
		case component::legacy_animation: return valid_fixed<payload::legacy_animation>(reader);
		case component::legacy_stars: return valid_stars(Model, Record, reader);
		case component::legacy_material: {
			payload::legacy_material material;
			return reader.get(material) && material.material < Model.materials.size();
		}
		default: return true; // a type of a later version, or a marker without data (LTXT)
	}
}

class validator {
  public:
	validator(model const &Model, std::size_t const Limit) : m_model(Model), m_limit(Limit) {}
	std::vector<std::string> run();

  private:
	bool full() const {
		return m_faults.size() >= m_limit;
	}
	void fault(std::string Text) {
		if (false == full())
			m_faults.push_back(std::move(Text));
	}
	void nodes();
	void node(std::uint32_t Index, node_record const &Node);
	void transforms();
	void meshes();
	void primitives();
	void primitive(std::uint32_t Index, primitive_record const &Primitive);
	void indices(std::uint32_t Index, primitive_record const &Primitive);
	void skinned(std::uint32_t Index, primitive_record const &Primitive);
	void bound(std::uint32_t Index, primitive_record const &Primitive);
	void slotted(std::uint32_t Index, primitive_record const &Primitive);
	void streams();
	void materials();
	void skins();
	void vertex_groups();
	void components();
	void component_ranges(std::uint32_t First, std::uint32_t Count, target_kind Kind, std::uint32_t Target, std::string_view Owner);
	void tables();

	model const &m_model;
	std::size_t m_limit;
	std::vector<std::string> m_faults;
};

void validator::node(std::uint32_t const Index, node_record const &Node) {
	auto const count{m_model.nodes.size()};
	if (false == valid_string(m_model, Node.name))
		fault(std::format("node {} has a name out of range", Index));
	auto const parent{Node.parent >= 0 && Node.parent < static_cast<std::int64_t>(Index)};
	if (Node.parent != -1 && false == parent)
		fault(std::format("node {} has parent {}, which isn't before it", Index, Node.parent));
	if (Node.subtree_end <= Index || Node.subtree_end > count)
		fault(std::format("node {} ends its subtree at {}", Index, Node.subtree_end));
	if (parent && Node.subtree_end > m_model.nodes[Node.parent].subtree_end)
		fault(std::format("node {} reaches past the subtree of its parent", Index));
	if (Node.mesh < -1 || Node.mesh >= static_cast<std::int64_t>(m_model.meshes.size()))
		fault(std::format("node {} has mesh {} out of range", Index, Node.mesh));
	if ((Node.flags & node_flags::matrix) != 0 && std::ranges::none_of(m_model.transforms, [Index](transform_record const &Transform) { return Transform.node == Index; }))
		fault(std::format("node {} has no matrix in XFRM", Index));
	if ((Node.flags & node_flags::matrix) != 0 && (Node.flags & node_flags::no_transform) != 0)
		fault(std::format("node {} has a matrix in XFRM and no transform at once", Index));
	component_ranges(Node.component_first, Node.component_count, target_kind::node, Index, std::format("node {}", Index));
}

void validator::nodes() {
	for (std::uint32_t idx = 0; idx < m_model.nodes.size() && false == full(); ++idx)
		node(idx, m_model.nodes[idx]);
	// a node within the subtree of another has it as an ancestor: the subtrees nest
	for (std::uint32_t idx = 0; idx < m_model.nodes.size() && false == full(); ++idx) {
		auto const parent{m_model.nodes[idx].parent};
		if (parent >= 0 && parent < static_cast<std::int64_t>(idx) && idx >= m_model.nodes[parent].subtree_end)
			fault(std::format("node {} lies outside the subtree of its parent", idx));
	}
}

void validator::transforms() {
	for (auto const &transform : m_model.transforms) {
		if (false == valid_node(m_model, transform.node))
			fault(std::format("a matrix in XFRM belongs to node {}, which isn't there", transform.node));
	}
}

void validator::meshes() {
	for (std::uint32_t idx = 0; idx < m_model.meshes.size() && false == full(); ++idx) {
		auto const &mesh{m_model.meshes[idx]};
		if (std::uint64_t{mesh.primitive_first} + mesh.primitive_count > m_model.primitives.size())
			fault(std::format("mesh {} has primitives out of range", idx));
		if (mesh.skin < -1 || mesh.skin >= static_cast<std::int64_t>(m_model.skins.size()))
			fault(std::format("mesh {} has skin {} out of range", idx, mesh.skin));
	}
}

void validator::indices(std::uint32_t const Index, primitive_record const &Primitive) {
	auto const wide{(Primitive.flags & primitive_flags::index32) != 0};
	auto const available{wide ? m_model.indices32.size() : m_model.indices16.size()};
	if (std::uint64_t{Primitive.first_index} + Primitive.index_count > available) {
		fault(std::format("primitive {} has indices out of range", Index));
		return;
	}
	for (std::size_t idx = Primitive.first_index; idx < std::size_t{Primitive.first_index} + Primitive.index_count; ++idx) {
		auto const value{wide ? m_model.indices32[idx] : std::uint32_t{m_model.indices16[idx]}};
		if (value >= Primitive.vertex_count) {
			fault(std::format("primitive {} has index {} past its {} vertices", Index, value, Primitive.vertex_count));
			return;
		}
	}
}

void validator::skinned(std::uint32_t const Index, primitive_record const &Primitive) {
	auto const &skin{m_model.skins[Primitive.skin]};
	auto const end{std::size_t{Primitive.base_vertex} + Primitive.vertex_count};
	auto const wide{m_model.weights_wide.size() == m_model.vertex_count()};
	for (auto idx = std::size_t{Primitive.base_vertex}; idx < end; ++idx) {
		auto const exceeds{[&skin](auto const &Joints) { return std::ranges::any_of(Joints, [&skin](auto const Joint) { return Joint >= skin.joint_count; }); }};
		auto const bad{wide ? exceeds(m_model.weights_wide[idx].joints) : exceeds(m_model.weights[idx].joints)};
		if (bad) {
			fault(std::format("primitive {} has vertex {} bound to a joint its skin doesn't have", Index, idx));
			return;
		}
	}
}

void validator::primitive(std::uint32_t const Index, primitive_record const &Primitive) {
	if (std::uint64_t{Primitive.base_vertex} + Primitive.vertex_count > m_model.vertex_count()) {
		fault(std::format("primitive {} has vertices out of range", Index));
		return;
	}
	if ((Primitive.flags & primitive_flags::topology_mask) > static_cast<std::uint32_t>(topology::triangle_fan))
		fault(std::format("primitive {} has an unknown topology", Index));
	if (false == valid_node(m_model, Primitive.node))
		fault(std::format("primitive {} belongs to node {}, which isn't there", Index, Primitive.node));
	if (Primitive.material != no_index && Primitive.material >= m_model.materials.size())
		fault(std::format("primitive {} has material {} out of range", Index, Primitive.material));
	if ((Primitive.flags & primitive_flags::indexed) != 0)
		indices(Index, Primitive);
	else if (std::uint64_t{Primitive.first_index} + Primitive.index_count > Primitive.vertex_count)
		fault(std::format("primitive {} draws vertices past its own", Index));
	if (Primitive.skin < -1 || Primitive.skin >= static_cast<std::int64_t>(m_model.skins.size()))
		fault(std::format("primitive {} has skin {} out of range", Index, Primitive.skin));
	else if ((Primitive.flags & primitive_flags::rigid_joints) != 0)
		bound(Index, Primitive);
	else if (Primitive.skin >= 0 && m_model.weights.size() != m_model.vertex_count() && m_model.weights_wide.size() != m_model.vertex_count())
		fault(std::format("primitive {} is skinned, and the model has no weights for its vertices", Index));
	else if (Primitive.skin >= 0)
		skinned(Index, Primitive);
	if ((Primitive.flags & primitive_flags::light_slots) != 0)
		slotted(Index, Primitive);
}

// a primitive of parts merged from several nodes: every vertex names a joint of the skin
void validator::bound(std::uint32_t const Index, primitive_record const &Primitive) {
	if (Primitive.skin < 0) {
		fault(std::format("primitive {} follows joints, and has no skin naming them", Index));
		return;
	}
	if (m_model.bindings.size() != m_model.vertex_count()) {
		fault(std::format("primitive {} follows joints, and the model has no VBND for its vertices", Index));
		return;
	}
	auto const &skin{m_model.skins[Primitive.skin]};
	auto const vertices{std::span{m_model.bindings}.subspan(Primitive.base_vertex, Primitive.vertex_count)};
	if (std::ranges::any_of(vertices, [&skin](vertex_binding const &Binding) { return Binding.joint >= skin.joint_count; }))
		fault(std::format("primitive {} has a vertex bound to a joint its skin doesn't have", Index));
}

void validator::slotted(std::uint32_t const Index, primitive_record const &Primitive) {
	if (m_model.bindings.size() != m_model.vertex_count()) {
		fault(std::format("primitive {} uses light slots, and the model has no VBND for its vertices", Index));
		return;
	}
	auto const vertices{std::span{m_model.bindings}.subspan(Primitive.base_vertex, Primitive.vertex_count)};
	if (std::ranges::any_of(vertices, [](vertex_binding const &Binding) { return Binding.role > slot_role::dark; }))
		fault(std::format("primitive {} has a vertex with an unknown light slot role", Index));
}

void validator::primitives() {
	for (std::uint32_t idx = 0; idx < m_model.primitives.size() && false == full(); ++idx)
		primitive(idx, m_model.primitives[idx]);
}

void validator::streams() {
	auto const count{m_model.vertex_count()};
	if (m_model.vertex_attributes.size() != count)
		fault(std::format("VATT has {} vertices, VPOS {}", m_model.vertex_attributes.size(), count));
	if (false == m_model.weights.empty() && m_model.weights.size() != count)
		fault(std::format("VSKN has {} vertices, VPOS {}", m_model.weights.size(), count));
	if (false == m_model.weights_wide.empty() && m_model.weights_wide.size() != count)
		fault(std::format("VSKW has {} vertices, VPOS {}", m_model.weights_wide.size(), count));
	if (false == m_model.weights.empty() && false == m_model.weights_wide.empty())
		fault("the model has both VSKN and VSKW");
	if (false == m_model.vertex_userdata.empty() && m_model.vertex_userdata.size() != count)
		fault(std::format("VUSR has {} vertices, VPOS {}", m_model.vertex_userdata.size(), count));
	if (false == m_model.bindings.empty() && m_model.bindings.size() != count)
		fault(std::format("VBND has {} vertices, VPOS {}", m_model.bindings.size(), count));
}

void validator::materials() {
	for (std::uint32_t idx = 0; idx < m_model.materials.size(); ++idx) {
		auto const &material{m_model.materials[idx]};
		if (false == valid_string(m_model, material.name) || material.replaceable < -4 || material.replaceable > 0)
			fault(std::format("material {} has a name or a skin out of range", idx));
	}
}

void validator::skins() {
	for (std::uint32_t idx = 0; idx < m_model.skins.size(); ++idx) {
		auto const &skin{m_model.skins[idx]};
		if (std::uint64_t{skin.joint_first} + skin.joint_count > m_model.skin_joints.size())
			fault(std::format("skin {} has joints out of range", idx));
		if (skin.root < -1 || skin.root >= static_cast<std::int64_t>(m_model.nodes.size()))
			fault(std::format("skin {} has root {} out of range", idx, skin.root));
		if (false == m_model.weights.empty() && skin.joint_count > 256)
			fault(std::format("skin {} has {} joints, VSKN can address 256", idx, skin.joint_count));
	}
	for (auto const &joint : m_model.skin_joints) {
		if (false == valid_node(m_model, joint.node))
			fault(std::format("a joint is node {}, which isn't there", joint.node));
	}
}

void validator::vertex_groups() {
	for (std::uint32_t idx = 0; idx < m_model.vertex_groups.size() && false == full(); ++idx) {
		auto const &group{m_model.vertex_groups[idx]};
		auto const ranges{(group.flags & vertex_group_flags::ranges) != 0};
		auto const itemsize{ranges ? 2u : 1u};
		auto const end{std::uint64_t{group.item_first} + std::uint64_t{group.item_count} * itemsize};
		if (group.mesh >= m_model.meshes.size() || false == valid_string(m_model, group.name) || end > m_model.vertex_group_items.size()) {
			fault(std::format("vertex group {} has its mesh, name or items out of range", idx));
			continue;
		}
		if ((group.flags & vertex_group_flags::weighted) != 0 && end > m_model.vertex_group_weights.size())
			fault(std::format("vertex group {} has weights out of range", idx));
		for (auto item = std::size_t{group.item_first}; item < end; item += itemsize) {
			auto const last{ranges ? std::uint64_t{m_model.vertex_group_items[item]} + m_model.vertex_group_items[item + 1] : std::uint64_t{m_model.vertex_group_items[item]} + 1};
			if (last > m_model.vertex_count()) {
				fault(std::format("vertex group {} has vertices out of range", idx));
				break;
			}
		}
		component_ranges(group.component_first, group.component_count, target_kind::vertex_group, idx, std::format("vertex group {}", idx));
	}
}

void validator::component_ranges(std::uint32_t const First, std::uint32_t const Count, target_kind const Kind, std::uint32_t const Target, std::string_view const Owner) {
	if (std::uint64_t{First} + Count > m_model.components.size()) {
		fault(std::format("{} has components out of range", Owner));
		return;
	}
	for (auto idx = First; idx < First + Count; ++idx) {
		if (m_model.components[idx].kind != Kind || m_model.components[idx].target != Target) {
			fault(std::format("{} lists component {}, which belongs to something else", Owner, idx));
			return;
		}
	}
}

void validator::components() {
	for (std::uint32_t idx = 0; idx < m_model.components.size() && false == full(); ++idx) {
		auto const &record{m_model.components[idx]};
		if (std::uint64_t{record.data_offset} + record.data_size > m_model.component_data.size() || record.data_offset % 4 != 0) {
			fault(std::format("component {} has its data out of range", idx));
			continue;
		}
		bool target{true};
		switch (record.kind) {
			case target_kind::model: target = record.target == 0; break;
			case target_kind::node: target = record.target < m_model.nodes.size(); break;
			case target_kind::mesh: target = record.target < m_model.meshes.size(); break;
			case target_kind::vertex_group: target = record.target < m_model.vertex_groups.size(); break;
			default: target = false; break;
		}
		if (false == target)
			fault(std::format("component {} ({}) belongs to something which isn't there", idx, fourcc_name(record.type)));
		else if (record.version == 1 && false == valid_payload(m_model, record, m_model.component_payload(idx)))
			fault(std::format("component {} ({}) has data which don't hold together", idx, fourcc_name(record.type)));
	}
}

void validator::tables() {
	if (false == m_model.bounds.empty() && m_model.bounds.size() != m_model.nodes.size() + 1)
		fault(std::format("BNDS has {} records for {} nodes", m_model.bounds.size(), m_model.nodes.size()));
	for (std::size_t idx = 0; idx < m_model.name_index.size(); ++idx) {
		auto const &entry{m_model.name_index[idx]};
		if (false == valid_node(m_model, entry.node) || (idx > 0 && entry.hash < m_model.name_index[idx - 1].hash)) {
			fault("the name index is out of order or out of range");
			break;
		}
	}
	for (auto const &edit : m_model.edit) {
		if (false == valid_string(m_model, edit.source) || false == valid_string(m_model, edit.tool))
			fault("EDIT has a string out of range");
	}
}

std::vector<std::string> validator::run() {
	nodes();
	transforms();
	streams();
	meshes();
	materials();
	skins();
	primitives();
	vertex_groups();
	components();
	tables();
	return m_faults;
}

} // namespace

std::vector<std::byte> write(model const &Model, write_options const &Options) {
	file_builder builder;
	builder.add(section::strings, true, section_alignment, encode_strings(Model.strings));
	builder.table(section::name_index, Model.name_index, false);
	builder.table(section::nodes, Model.nodes, true);
	builder.table(section::transforms, Model.transforms, false);
	builder.table(section::meshes, Model.meshes, true);
	builder.table(section::primitives, Model.primitives, true);
	builder.table(section::materials, Model.materials, true);
	builder.table(section::skins, Model.skins, false);
	builder.table(section::skin_joints, Model.skin_joints, false);
	builder.table(section::vertex_groups, Model.vertex_groups, false);
	builder.stream(section::vertex_group_items, Model.vertex_group_items, false);
	builder.stream(section::vertex_group_weights, Model.vertex_group_weights, false);
	builder.table(section::components, Model.components, false);
	if (false == Model.component_data.empty())
		builder.add(section::component_data, false, section_alignment, Model.component_data);
	builder.table(section::bounds, Model.bounds, false);
	builder.table(section::edit, Model.edit, false);
	builder.stream(section::positions, Model.positions, true, stream_codec::vertices);
	builder.stream(section::attributes, Model.vertex_attributes, true, stream_codec::vertices);
	builder.stream(section::skin_weights, Model.weights, false, stream_codec::vertices);
	builder.stream(section::skin_weights_wide, Model.weights_wide, false, stream_codec::vertices);
	builder.stream(section::userdata, Model.vertex_userdata, false, stream_codec::vertices);
	// required once present: the primitives which follow it draw wrong without it
	builder.stream(section::bindings, Model.bindings, false == Model.bindings.empty(), stream_codec::vertices);
	builder.stream(section::indices16, Model.indices16, false, stream_codec::indices);
	builder.stream(section::indices32, Model.indices32, false, stream_codec::indices);
	for (auto const &section : Model.unknown)
		builder.raw(section.id, section.flags, section.data);
	return builder.finish(Model, Options);
}

read_result read(std::span<std::byte const> const File, model &Model, read_options const &Options) {
	return file_reader{File, Options}.run(Model);
}

std::vector<std::string> validate(model const &Model, std::size_t const Limit) {
	return validator{Model, Limit}.run();
}

} // namespace e4d
