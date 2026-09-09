module;
#include <memory>
#include <string>
#include <utility>
#include <map>
#include <vector>
#include <filesystem>

export module eu07.launcher.scenery_scanner;
import eu07.global_include.interfaces.itexture;
import eu07.model.texture;
import eu07.utilities.parser;
import eu07.launcher.textures_scanner;

export {


struct dynamic_desc {
	std::string name;
	std::string drivertype = "nobody";

	float offset;
	std::string loadtype = "none";
	int loadcount;

	unsigned int coupling;
	std::string params;

	std::shared_ptr<ui::vehicle_desc> vehicle;
	std::shared_ptr<ui::skin_set> skin;
};

struct trainset_desc {
	std::pair<int, int> file_bounds;

	std::string description;

	std::string name;
	std::string track;

	float offset { 0.f };
	float velocity { 0.f };

	std::vector<dynamic_desc> vehicles;
};

struct scenery_desc {
	std::filesystem::path path;
	std::string name;
	std::string description;
	std::string category;
	std::string image_path;
	texture_handle image = 0;

	std::vector<std::pair<std::string, std::string>> links;
	std::vector<trainset_desc> trainsets;

	std::string title() {
		if (!name.empty())
			return name;
		else
			return path.stem().string();
	}
};

class scenery_scanner {
public:
	scenery_scanner(ui::vehicles_bank &bank);

	std::vector<scenery_desc> scenarios;
	std::map<std::string, std::vector<scenery_desc*>> categories;

	void scan();

private:
	void scan_scn(std::filesystem::path path);
	void parse_trainset(cParser &parser);
	void build_categories();

	ui::vehicles_bank &bank;
};

}  // export
