#pragma once

// Jednorazowy indeks nagłówków plików ASC (NMT1) — bbox w PUWG, bez wczytywania siatki.

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace eu07::nmt {

inline constexpr std::string_view kAscIndexMagic = "# parser_nmt_index v1";

struct AscHeader {
    int ncols = 0;
    int nrows = 0;
    double xll = 0.0;
    double yll = 0.0;
    double cellsize = 0.0;
    double nodata = -9999.0;
};

struct AscIndexEntry {
    std::filesystem::path relativePath;
    std::int64_t mtimeUtc = 0;
    AscHeader header{};
    double northMin = 0.0;
    double northMax = 0.0;
    double eastMin = 0.0;
    double eastMax = 0.0;
};

struct AscIndex {
    std::filesystem::path catalogRoot;
    std::vector<AscIndexEntry> entries;
};

[[nodiscard]] inline std::filesystem::path defaultIndexPath(const std::filesystem::path& catalogRoot) {
    return catalogRoot / ".parser_nmt_index.tsv";
}

[[nodiscard]] inline std::int64_t fileMtimeUtc(const std::filesystem::path& path) {
    std::error_code ec;
    const auto ftime = std::filesystem::last_write_time(path, ec);
    if (ec) {
        return 0;
    }
    const auto sctp = std::chrono::time_point_cast<std::chrono::seconds>(
        ftime - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
    return std::chrono::duration_cast<std::chrono::seconds>(sctp.time_since_epoch()).count();
}

[[nodiscard]] inline bool parseAscHeaderLine(
    std::string_view line,
    std::string_view key,
    double& outValue) {
    if (!line.starts_with(key)) {
        return false;
    }
    const std::size_t pos = line.find_first_of(" \t");
    if (pos == std::string_view::npos) {
        return false;
    }
    std::string_view tail = line.substr(pos);
    while (!tail.empty() && (tail.front() == ' ' || tail.front() == '\t')) {
        tail.remove_prefix(1);
    }
    try {
        outValue = std::stod(std::string(tail));
        return true;
    } catch (...) {
        return false;
    }
}

[[nodiscard]] inline bool parseAscHeaderLine(
    std::string_view line,
    std::string_view key,
    int& outValue) {
    double tmp = 0.0;
    if (!parseAscHeaderLine(line, key, tmp)) {
        return false;
    }
    outValue = static_cast<int>(tmp);
    return true;
}

[[nodiscard]] inline std::optional<AscHeader> readAscHeaderOnly(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        return std::nullopt;
    }

    AscHeader header;
    std::string line;
    int parsedLines = 0;
    while (parsedLines < 6 && std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        if (parseAscHeaderLine(line, "ncols", header.ncols) ||
            parseAscHeaderLine(line, "NCOLS", header.ncols)) {
            ++parsedLines;
            continue;
        }
        if (parseAscHeaderLine(line, "nrows", header.nrows) ||
            parseAscHeaderLine(line, "NROWS", header.nrows)) {
            ++parsedLines;
            continue;
        }
        if (parseAscHeaderLine(line, "xllcorner", header.xll) ||
            parseAscHeaderLine(line, "XLLCORNER", header.xll) ||
            parseAscHeaderLine(line, "xllcenter", header.xll) ||
            parseAscHeaderLine(line, "XLLCENTER", header.xll)) {
            ++parsedLines;
            continue;
        }
        if (parseAscHeaderLine(line, "yllcorner", header.yll) ||
            parseAscHeaderLine(line, "YLLCORNER", header.yll) ||
            parseAscHeaderLine(line, "yllcenter", header.yll) ||
            parseAscHeaderLine(line, "YLLCENTER", header.yll)) {
            ++parsedLines;
            continue;
        }
        if (parseAscHeaderLine(line, "cellsize", header.cellsize) ||
            parseAscHeaderLine(line, "CELLSIZE", header.cellsize)) {
            ++parsedLines;
            continue;
        }
        if (parseAscHeaderLine(line, "NODATA_value", header.nodata) ||
            parseAscHeaderLine(line, "nodata_value", header.nodata) ||
            parseAscHeaderLine(line, "NODATA", header.nodata)) {
            ++parsedLines;
            continue;
        }
    }

    if (header.ncols <= 0 || header.nrows <= 0 || header.cellsize <= 0.0) {
        return std::nullopt;
    }
    return header;
}

[[nodiscard]] inline AscIndexEntry makeIndexEntry(
    const std::filesystem::path& catalogRoot,
    const std::filesystem::path& ascPath,
    const AscHeader& header) {
    AscIndexEntry entry;
    entry.relativePath = std::filesystem::relative(ascPath, catalogRoot);
    entry.mtimeUtc = fileMtimeUtc(ascPath);
    entry.header = header;
    // Konwencja ESRI ASC + PUWG (jak terenAI): xll/yll = easting/northing rogu siatki.
    entry.northMin = header.yll;
    entry.northMax = header.yll + header.nrows * header.cellsize;
    entry.eastMin = header.xll;
    entry.eastMax = header.xll + header.ncols * header.cellsize;
    return entry;
}

[[nodiscard]] inline bool bboxIntersects(
    const double northMin,
    const double northMax,
    const double eastMin,
    const double eastMax,
    const AscIndexEntry& entry) {
    return entry.northMax >= northMin && entry.northMin <= northMax && entry.eastMax >= eastMin &&
           entry.eastMin <= eastMax;
}

[[nodiscard]] inline AscIndex buildAscIndex(const std::filesystem::path& catalogRoot) {
    AscIndex index;
    index.catalogRoot = std::filesystem::absolute(catalogRoot);

    std::error_code ec;
    if (!std::filesystem::is_directory(index.catalogRoot, ec)) {
        throw std::runtime_error("Katalog NMT nie istnieje: " + index.catalogRoot.string());
    }

    for (const auto& entry : std::filesystem::recursive_directory_iterator(
             index.catalogRoot, std::filesystem::directory_options::skip_permission_denied, ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::filesystem::path path = entry.path();
        const std::string ext = path.extension().string();
        if (ext != ".asc" && ext != ".ASC") {
            continue;
        }
        if (path.filename() == ".parser_nmt_index.tsv") {
            continue;
        }

        const std::optional<AscHeader> header = readAscHeaderOnly(path);
        if (!header) {
            continue;
        }
        index.entries.push_back(makeIndexEntry(index.catalogRoot, path, *header));
    }

    return index;
}

inline void writeAscIndex(const AscIndex& index, const std::filesystem::path& outPath) {
    std::ofstream out(outPath);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac indeksu: " + outPath.string());
    }

    out << kAscIndexMagic << '\n';
    out << "# catalog=" << index.catalogRoot.string() << '\n';
    out << "# path\tmtime\tncols\tnrows\txll\tyll\tcellsize\tnodata\tnorth_min\tnorth_max\teast_min\teast_max\n";

    for (const AscIndexEntry& e : index.entries) {
        out << e.relativePath.generic_string() << '\t' << e.mtimeUtc << '\t' << e.header.ncols << '\t'
            << e.header.nrows << '\t' << e.header.xll << '\t' << e.header.yll << '\t' << e.header.cellsize
            << '\t' << e.header.nodata << '\t' << e.northMin << '\t' << e.northMax << '\t' << e.eastMin
            << '\t' << e.eastMax << '\n';
    }
}

[[nodiscard]] inline std::optional<AscIndex> loadAscIndex(const std::filesystem::path& indexPath) {
    std::ifstream in(indexPath);
    if (!in) {
        return std::nullopt;
    }

    std::string line;
    if (!std::getline(in, line) || line != kAscIndexMagic) {
        return std::nullopt;
    }

    AscIndex index;
    while (std::getline(in, line)) {
        if (line.empty() || line.starts_with('#')) {
            if (line.starts_with("# catalog=")) {
                index.catalogRoot = line.substr(std::string_view("# catalog=").size());
            }
            continue;
        }

        std::stringstream ss(line);
        AscIndexEntry entry;
        std::string rel;
        if (!(ss >> rel >> entry.mtimeUtc >> entry.header.ncols >> entry.header.nrows >> entry.header.xll >>
              entry.header.yll >> entry.header.cellsize >> entry.header.nodata >> entry.northMin >>
              entry.northMax >> entry.eastMin >> entry.eastMax)) {
            continue;
        }
        entry.relativePath = rel;
        index.entries.push_back(std::move(entry));
    }
    return index;
}

[[nodiscard]] inline bool ascIndexEntryStale(
    const AscIndexEntry& entry,
    const std::filesystem::path& catalogRoot) {
    const std::filesystem::path full = catalogRoot / entry.relativePath;
    return fileMtimeUtc(full) != entry.mtimeUtc;
}

enum class AscIndexStatus { Loaded, Created, Refreshed };

struct AscIndexResult {
    AscIndex index;
    AscIndexStatus status = AscIndexStatus::Loaded;
};

[[nodiscard]] inline AscIndexResult loadOrBuildAscIndex(const std::filesystem::path& catalogRoot) {
    const std::filesystem::path indexPath = defaultIndexPath(catalogRoot);

    if (const std::optional<AscIndex> loaded = loadAscIndex(indexPath)) {
        AscIndex index = *loaded;
        if (index.catalogRoot.empty()) {
            index.catalogRoot = std::filesystem::absolute(catalogRoot);
        }
        bool stale = false;
        for (const AscIndexEntry& e : index.entries) {
            if (ascIndexEntryStale(e, index.catalogRoot)) {
                stale = true;
                break;
            }
        }
        if (!stale) {
            return {std::move(index), AscIndexStatus::Loaded};
        }
    }

    const bool hadIndexFile = std::filesystem::exists(indexPath);
    AscIndex built = buildAscIndex(catalogRoot);
    writeAscIndex(built, indexPath);
    return {
        std::move(built),
        hadIndexFile ? AscIndexStatus::Refreshed : AscIndexStatus::Created,
    };
}

[[nodiscard]] inline std::vector<const AscIndexEntry*> selectEntriesForCorridor(
    const AscIndex& index,
    const double northMin,
    const double northMax,
    const double eastMin,
    const double eastMax) {
    std::vector<const AscIndexEntry*> selected;
    selected.reserve(index.entries.size());
    for (const AscIndexEntry& e : index.entries) {
        if (bboxIntersects(northMin, northMax, eastMin, eastMax, e)) {
            selected.push_back(&e);
        }
    }
    return selected;
}

} // namespace eu07::nmt
