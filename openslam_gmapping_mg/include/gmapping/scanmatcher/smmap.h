#ifndef SMMAP_H
#define SMMAP_H
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/point.h>
#define SIGHT_INC 1

namespace GMapping {

struct PointAccumulator{
	typedef point<float> FloatPoint;
	/* before 
	PointAccumulator(int i=-1): acc(0,0), n(0), visits(0){assert(i==-1);}
	*/
	/*after begin*/
	PointAccumulator(): acc(0,0), n(0), visits(0){}
	PointAccumulator(int i): acc(0,0), n(0), visits(0){assert(i==-1);}
	/*after end*/
        inline void update(bool value, const Point& p=Point(0,0));
	inline Point mean() const {return 1./n*Point(acc.x, acc.y);}
	inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
	inline void add(const PointAccumulator& p) {acc=acc+p.acc; n+=p.n; visits+=p.visits; }
	static const PointAccumulator& Unknown();
	static PointAccumulator* unknown_ptr;
	FloatPoint acc;
	int n, visits;
	inline double entropy() const;
};

void PointAccumulator::update(bool value, const Point& p){
	if (value) {
		acc.x+= static_cast<float>(p.x);
		acc.y+= static_cast<float>(p.y); 
		n++; 
		visits+=SIGHT_INC;
	} else
		visits++;
}

double PointAccumulator::entropy() const{
	if (!visits)
		return -log(.5);
	if (n==visits || n==0)
		return 0;
	double x=(double)n*SIGHT_INC/(double)visits;
	return -( x*log(x)+ (1-x)*log(1-x) );
}


typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;


//mg added
struct Vec3{
	double x,y,z;
	inline Vec3():x(0), y(0), z(0){};
	inline Vec3(double x, double y, double z):x(x), y(y), z(z){};
	inline Vec3 operator + (const Vec3& vec) const {return Vec3(x + vec.x, y+vec.y, z+vec.z);}
	inline Vec3 operator - (const Vec3& vec) const {return Vec3(x - vec.x, y - vec.y, z - vec.z);}
	inline Vec3 operator / (const float n) const {return Vec3(x / n, y / n, z / n);}
	inline Vec3 operator * (const float n) const {return Vec3(x * n, y * n, z * n);}
};

struct PointMg{
	PointMg(): vec(Vec3()), n(0), visits(0){}
	PointMg(int i): vec(Vec3()), n(0), visits(0){assert(i==-1);}
	/*after end*/
        inline void update(bool value, const Vec3& p=Vec3());
	inline Vec3 mean() const {
		return Vec3(vec.x/n, vec.y/n, vec.z/n);
	}
	inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
	inline void add(const PointMg& p) {vec=vec+p.vec; n+=p.n; visits+=p.visits; }
	static const PointMg& Unknown();
	static PointMg* unknown_ptr;
	Vec3 vec;
	int n, visits;
	inline double entropy() const;
};

void PointMg::update(bool value, const Vec3& p){
	if (value) {
		vec.x+= p.x;
		vec.y+= p.y;
		vec.z+= p.z;
		n++; 
		visits+=SIGHT_INC;
	} else
		visits++;
}

double PointMg::entropy() const{
	if (!visits)
		return -log(.5);
	if (n==visits || n==0)
		return 0;
	double x=(double)n*SIGHT_INC/(double)visits;
	return -( x*log(x)+ (1-x)*log(1-x) );
}

typedef Map<PointMg,HierarchicalArray2D<PointMg> > MgMatcherMap;

};

#endif 
