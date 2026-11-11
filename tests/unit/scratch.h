#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>

namespace test {

// a folder of its own for the files of a test, removed with the object. it's made in the build folder of the tests
// (EU07_UNIT_TESTS_SCRATCH, see CMakeLists.txt), not in the system's temporary folder, which anyone can write to
class scratch_folder {
  public:
	explicit scratch_folder(std::string const &Name) : m_root{std::filesystem::path{EU07_UNIT_TESTS_SCRATCH} / ("eu07_unit_test_" + Name)} {
		std::filesystem::remove_all(m_root);
		std::filesystem::create_directories(m_root);
	}
	~scratch_folder() {
		std::error_code error;
		std::filesystem::remove_all(m_root, error);
	}
	scratch_folder(scratch_folder const &) = delete;
	scratch_folder &operator=(scratch_folder const &) = delete;

	// writes the file byte for byte, so the line ends are the ones the test gives. returns its path, with forward slashes
	std::string write(std::string const &Name, std::string_view const Content) const {
		auto const path{m_root / Name};
		std::filesystem::create_directories(path.parent_path());
		std::ofstream file{path, std::ios::binary};
		file.write(Content.data(), static_cast<std::streamsize>(Content.size()));
		return path.generic_string();
	}
	// the folder with forward slashes and one at the end, the way the game names the folder of a file to the parser
	std::string path() const { return m_root.generic_string() + "/"; }
	// a file or a folder of that name in the folder, with forward slashes
	std::string path(std::string const &Name) const { return (m_root / Name).generic_string(); }
	std::filesystem::path const &root() const { return m_root; }

  private:
	std::filesystem::path m_root;
};

} // namespace test
