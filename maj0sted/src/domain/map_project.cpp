#include "maj0sted/domain/map_project.hpp"

#include <utility>

namespace maj0sted::domain {

MapProject::MapProject(Crs crs) : crs_{crs} {}

void MapProject::add_niweleta(Niweleta niweleta) {
    niwelety_.push_back(std::move(niweleta));
}

}  // namespace maj0sted::domain
