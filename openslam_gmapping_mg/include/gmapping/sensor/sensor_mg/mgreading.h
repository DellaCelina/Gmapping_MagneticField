#ifndef MGREADING_H
#define MGREADING_H

#include <vector>
#include <gmapping/sensor/sensor_base/sensorreading.h>
#include <gmapping/sensor/sensor_mg/mgsensor.h>

namespace GMapping{
struct vec3{
	double x, y, z;
	vec3():x(0), y(0), z(0){}
	vec3(double x, double y, double z): x(x), y(y), z(z){}
};
	
class MgReading: public SensorReading, public std::vector<vec3>{
	public:
		MgReading(const MgSensor* mg, double time=0);
		MgReading(const double (*vec)[3], const MgSensor* mg, double time=0);
		inline const OrientedPoint& getPose() const {return m_pose;}
		inline void setPose(const OrientedPoint& pose) {m_pose = pose;}
	protected:
		OrientedPoint m_pose;
};

};

#endif
