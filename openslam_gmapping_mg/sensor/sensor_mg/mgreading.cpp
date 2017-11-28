#include <gmapping/sensor/sensor_mg/mgreading.h>

namespace GMapping{
using namespace std;

MgReading::MgReading(const MgSensor* mg, double time):SensorReading(mg, time){}
MgReading::MgReading(const double (*vec)[3], const MgSensor* mg, double time):SensorReading(mg, time){
	for(int i = 0; i<5; i++){
		(*this).push_back(vec3(vec[i][0], vec[i][1], vec[i][2]));
	}
}

};
