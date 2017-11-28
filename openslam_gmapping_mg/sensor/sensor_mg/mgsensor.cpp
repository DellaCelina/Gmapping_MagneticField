#include <gmapping/sensor/sensor_mg/mgsensor.h>

namespace GMapping{

		MgSensor::MgSensor(std::string name, double distance, const OrientedPoint& position):Sensor(name), m_pose(position), m_distance(distance){}

};
