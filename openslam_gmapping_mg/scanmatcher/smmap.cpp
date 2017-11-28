#include <gmapping/scanmatcher/smmap.h>

namespace GMapping {

const PointAccumulator& PointAccumulator::Unknown(){
	if (! unknown_ptr)
		unknown_ptr=new PointAccumulator;
	return *unknown_ptr;
}

PointAccumulator* PointAccumulator::unknown_ptr=0;

const PointMg& PointMg::Unknown(){
	if(!unknown_ptr)
		unknown_ptr=new PointMg;
	return *unknown_ptr;
}
PointMg* PointMg::unknown_ptr=0;
};


