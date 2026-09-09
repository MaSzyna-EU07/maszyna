module;
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

export module eu07.utilities.headtrack;

export {

class headtrack
{
    int joy_id = -1;

    void find_joy();
    float get_axis(const float *data, int count, int axis, float mul);

public:
    headtrack();

    void update();
};

}  // export
