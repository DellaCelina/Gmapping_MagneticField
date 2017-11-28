#ifndef MGSENSOR_H
#define MGSENSOR_H
#include <vector>
#include <gmapping/sensor/sensor_base/sensor.h>
#include <gmapping/utils/point.h>

namespace GMapping{
class MgSensor: public Sensor{
	public:
		MgSensor(std::string name, double distance, const OrientedPoint& position=OrientedPoint(0,0,0)) ;
		inline OrientedPoint getPose() const {return m_pose;}
		inline int distance() const {return m_distance;}
	protected:
		OrientedPoint m_pose;
		double m_distance;
};

};

#endif
