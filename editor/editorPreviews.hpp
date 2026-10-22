/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// preview images of the models of the node bank: made by -generate-nodebank-previews, shown by the node bank of the editor.
// both find an image by the same name, made here
namespace editor_previews
{

// folder of the images, under the folder of the simulator
extern std::string const folder;

// characters which can be safely used in a file name, the others replaced by _
std::string file_name_part(std::string Text);
// path of the image of the node with specified model and skin, without the extension: the path of the model in the preview folder,
// the skin added to the name
std::string preview_path(std::string Model, std::string const &Skin);
// path of the image of a node bank entry ("node ... model <x> <y> <z> <angle> <model> <skin> ..."), without the extension;
// empty if the entry isn't a model
std::string entry_preview_path(std::string const &Entry);
// model and skin of a node bank entry, as above. returns: false if the entry isn't a model
bool entry_model(std::string const &Entry, std::string &Model, std::string &Skin);
// name given to another entry with the same model and skin (other lights or angles): ~2, ~3... added in the order of the
// node bank, after the names already Taken
std::string variant(std::string const &Path, std::set<std::string> const &Taken);

// images of the node bank as textures of the user interface, small and kept for a while: the files are read and scaled down
// in a thread of their own, and uploaded in the main thread, a few in a frame
class image_cache
{
  public:
	// Size: pixels of the longer side of the images, 0 for the size of the cards of the node bank; Limit: images kept at most
	explicit image_cache(int const Size = 0, std::size_t const Limit = 512);
	~image_cache();
	image_cache(image_cache const &) = delete;
	image_cache &operator=(image_cache const &) = delete;

	// texture of the image at specified path (with the extension), 0 while it's being read or when there's no such image.
	// an image not asked for longer than others is dropped when there are too many
	std::uint64_t image(std::string const &Path);
	// true if there's no image at specified path
	bool missing(std::string const &Path) const;
	// to be called once in a frame: uploads the images read since the last frame
	void update();
	// drops all images, so that they're read again (e.g. after new ones were generated)
	void clear();

  private:
	struct entry
	{
		std::uint64_t texture{0};
		bool missing{false};
		bool pending{false}; // waits in the queue or is being read
		std::uint64_t used{0}; // frame it was asked for in the last time
	};
	struct decoded
	{
		std::string path;
		int width{0};
		int height{0};
		std::vector<std::uint8_t> rgba; // empty if there's no such image
		unsigned int generation{0};
	};
	void work();
	void release(entry &Entry);
	// members
	std::unordered_map<std::string, entry> m_entries;
	std::uint64_t m_frame{0};
	int m_size{128}; // pixels of the longer side of the images
	std::size_t m_limit{512};
	// shared with the thread
	std::mutex m_lock;
	std::condition_variable m_wake;
	std::deque<std::string> m_queue; // the image asked for in the last is read first, it's the one on the screen
	std::vector<decoded> m_done;
	unsigned int m_generation{0}; // images read before the last clear() are thrown away
	bool m_quit{false};
	std::thread m_thread;
};

} // namespace editor_previews
