module;
#include <string>
#include <chrono>

export module eu07.utilities.motiontelemetry;
import eu07.glm;

export {


class motiontelemetry
{
public:
	struct conf_t
	{
		bool enable;
		std::string proto;
		std::string address;
		std::string port;
		float updatetime;
		bool includegravity;
		bool fwdposbased;
		bool latposbased;
		float axlebumpscale;
	};

private:
	int sock;
	std::chrono::time_point<std::chrono::high_resolution_clock> last_update;
	conf_t conf;
	glm::dvec3 last_pos;
	glm::dvec3 last_vel;
	double last_yaw;

public:
	motiontelemetry();
	~motiontelemetry();
	void update();
};

}  // export
