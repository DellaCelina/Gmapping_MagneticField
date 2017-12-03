#include <string>
#include <sstream>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>
#include <gmapping/utils/stat.h>
#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <gmapping/scanmatcher/smmap.h>

//#define MAP_CONSISTENCY_CHECK
//#define GENERATE_TRAJECTORIES

namespace GMapping {

const double m_distanceThresholdCheck = 20;
 
using namespace std;

  GridSlamProcessor::GridSlamProcessor(): m_infoStream(cout){
    
    period_ = 5.0;
    m_obsSigmaGain=1;
    m_resampleThreshold=0.5;
    m_minimumScore=0.;
  }
  
  GridSlamProcessor::GridSlamProcessor(const GridSlamProcessor& gsp) 
    :last_update_time_(0.0), m_particles(gsp.m_particles), m_infoStream(cout){
    
    period_ = 5.0;

    m_obsSigmaGain=gsp.m_obsSigmaGain;
    m_resampleThreshold=gsp.m_resampleThreshold;
    m_minimumScore=gsp.m_minimumScore;
    
    m_beams=gsp.m_beams;
    m_indexes=gsp.m_indexes;
    m_motionModel=gsp.m_motionModel;
    m_resampleThreshold=gsp.m_resampleThreshold;
    m_matcher=gsp.m_matcher;
    
    m_count=gsp.m_count;
    m_readingCount=gsp.m_readingCount;
    m_lastPartPose=gsp.m_lastPartPose;
    m_pose=gsp.m_pose;
    m_odoPose=gsp.m_odoPose;
    m_linearDistance=gsp.m_linearDistance;
    m_angularDistance=gsp.m_angularDistance;
    m_neff=gsp.m_neff;
	
    cerr << "FILTER COPY CONSTRUCTOR" << endl;
    cerr << "m_odoPose=" << m_odoPose.x << " " <<m_odoPose.y << " " << m_odoPose.theta << endl;
    cerr << "m_lastPartPose=" << m_lastPartPose.x << " " <<m_lastPartPose.y << " " << m_lastPartPose.theta << endl;
    cerr << "m_linearDistance=" << m_linearDistance << endl;
    cerr << "m_angularDistance=" << m_linearDistance << endl;
    
		
    m_xmin=gsp.m_xmin;
    m_ymin=gsp.m_ymin;
    m_xmax=gsp.m_xmax;
    m_ymax=gsp.m_ymax;
    m_delta=gsp.m_delta;
    
    m_regScore=gsp.m_regScore;
    m_critScore=gsp.m_critScore;
    m_maxMove=gsp.m_maxMove;
    
    m_linearThresholdDistance=gsp.m_linearThresholdDistance;
    m_angularThresholdDistance=gsp.m_angularThresholdDistance;
    m_obsSigmaGain=gsp.m_obsSigmaGain;
    
#ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ <<  ": trajectories copy.... ";
#endif
    TNodeVector v=gsp.getTrajectories();
    for (unsigned int i=0; i<v.size(); i++){
		m_particles[i].node=v[i];
    }
#ifdef MAP_CONSISTENCY_CHECK
    cerr <<  "end" << endl;
#endif


    cerr  << "Tree: normalizing, resetting and propagating weights within copy construction/cloneing ..." ;
    updateTreeWeights(false);
    cerr  << ".done!" <<endl;
  }
  
  GridSlamProcessor::GridSlamProcessor(std::ostream& infoS): m_infoStream(infoS){
    period_ = 5.0;
    m_obsSigmaGain=1;
    m_resampleThreshold=0.5;
    m_minimumScore=0.;
	
  }

  GridSlamProcessor* GridSlamProcessor::clone() const {
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing preclone_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
	for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	  const ScanMatcherMap& m1(it->map);
	  const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
 	  for (int x=0; x<h1.getXSize(); x++){
	    for (int y=0; y<h1.getYSize(); y++){
	      const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	      if (a1.m_reference){
		PointerMap::iterator f=pmap.find(a1.m_reference);
		if (f==pmap.end())
		  pmap.insert(make_pair(a1.m_reference, 1));
		else
		  f->second++;
	      }
	    }
	  }
	}
	cerr << __PRETTY_FUNCTION__ <<  ": Number of allocated chunks" << pmap.size() << endl;
	for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
	  assert(it->first->shares==(unsigned int)it->second);

	cerr << __PRETTY_FUNCTION__ <<  ": SUCCESS, the error is somewhere else" << endl;
# endif
	GridSlamProcessor* cloned=new GridSlamProcessor(*this);
	
# ifdef MAP_CONSISTENCY_CHECK
	cerr << __PRETTY_FUNCTION__ <<  ": trajectories end" << endl;
	cerr << __PRETTY_FUNCTION__ << ": performing afterclone_fit_test" << endl;
	ParticleVector::const_iterator jt=cloned->m_particles.begin();
	for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	  const ScanMatcherMap& m1(it->map);
	  const ScanMatcherMap& m2(jt->map);
	  const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
	  const HierarchicalArray2D<PointAccumulator>& h2(m2.storage());
	  jt++;
 	  for (int x=0; x<h1.getXSize(); x++){
	    for (int y=0; y<h1.getYSize(); y++){
	      const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	      const autoptr< Array2D<PointAccumulator> >& a2(h2.m_cells[x][y]);
	      assert(a1.m_reference==a2.m_reference);
	      assert((!a1.m_reference) || !(a1.m_reference->shares%2));
	    }
	  }
	}
	cerr << __PRETTY_FUNCTION__ <<  ": SUCCESS, the error is somewhere else" << endl;
# endif
	return cloned;
}
  
  GridSlamProcessor::~GridSlamProcessor(){
    cerr << __PRETTY_FUNCTION__ << ": Start" << endl;
    cerr << __PRETTY_FUNCTION__ << ": Deleting tree" << endl;
    for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
#ifdef TREE_CONSISTENCY_CHECK		
      TNode* node=it->node;
      while(node)
	node=node->parent;
      cerr << "@" << endl;
#endif
      if (it->node)
	delete it->node;
      //cout << "l=" << it->weight<< endl;
    }
    
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing predestruction_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      const ScanMatcherMap& m1(it->map);
      const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
      for (int x=0; x<h1.getXSize(); x++){
	for (int y=0; y<h1.getYSize(); y++){
	  const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	  if (a1.m_reference){
	    PointerMap::iterator f=pmap.find(a1.m_reference);
	    if (f==pmap.end())
	      pmap.insert(make_pair(a1.m_reference, 1));
	    else
	      f->second++;
	  }
	}
      }
    }
    cerr << __PRETTY_FUNCTION__ << ": Number of allocated chunks" << pmap.size() << endl;
    for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
      assert(it->first->shares>=(unsigned int)it->second);
    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
  }


		
  void GridSlamProcessor::setMatchingParameters (double urange, double range, double sigma, int kernsize, double lopt, double aopt, 
						 int iterations, double likelihoodSigma, double likelihoodGain, unsigned int likelihoodSkip){
    m_obsSigmaGain=likelihoodGain;
    m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
    if (m_infoStream)
      m_infoStream << " -maxUrange "<< urange
		   << " -maxUrange "<< range
		   << " -sigma     "<< sigma
		   << " -kernelSize "<< kernsize
		   << " -lstep "    << lopt
		   << " -lobsGain " << m_obsSigmaGain
		   << " -astep "    << aopt << endl;
    
    
  }
  
void GridSlamProcessor::setMotionModelParameters
(double srr, double srt, double str, double stt){
  m_motionModel.srr=srr;
  m_motionModel.srt=srt;
  m_motionModel.str=str;
  m_motionModel.stt=stt;	
  
  if (m_infoStream)
    m_infoStream << " -srr "<< srr 	<< " -srt "<< srt  
		 << " -str "<< str 	<< " -stt "<< stt << endl;
  
}
  
  void GridSlamProcessor::setUpdateDistances(double linear, double angular, double resampleThreshold){
    m_linearThresholdDistance=linear; 
    m_angularThresholdDistance=angular;
    m_resampleThreshold=resampleThreshold;	
    if (m_infoStream)
      m_infoStream << " -linearUpdate " << linear
		   << " -angularUpdate "<< angular
		   << " -resampleThreshold " << m_resampleThreshold << endl;
  }
  
  //HERE STARTS THE BEEF
  GridSlamProcessor::Particle::Particle(const ScanMatcherMap& m, const MgMatcherMap& map_mg):
  map_mg(map_mg), map(m), pose(0,0,0), weight(0), weightSum(0), gweight(0), previousIndex(0){
  node=0;
}
  
  void GridSlamProcessor::setSensorMap(const SensorMap& smap){
    
    /*
      Construct the angle table for the sensor
      
      FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot 
    */
    
    SensorMap::const_iterator laser_it=smap.find(std::string("FLASER"));
    if (laser_it==smap.end()){
      cerr << "Attempting to load the new carmen log format" << endl;
      laser_it=smap.find(std::string("ROBOTLASER1"));
      assert(laser_it!=smap.end());
    }
    const RangeSensor* rangeSensor=dynamic_cast<const RangeSensor*>((laser_it->second));
    assert(rangeSensor && rangeSensor->beams().size());
    
    m_beams=static_cast<unsigned int>(rangeSensor->beams().size());
    double* angles=new double[rangeSensor->beams().size()];
    for (unsigned int i=0; i<m_beams; i++){
      angles[i]=rangeSensor->beams()[i].pose.theta;
    }
    m_matcher.setLaserParameters(m_beams, angles, rangeSensor->getPose());
    delete [] angles;
  }

  void GridSlamProcessor::setSensorMap_mg(const SensorMap& smap){
    
    /*
      Construct the angle table for the sensor
      
      FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot 
    */
    
    SensorMap::const_iterator mg_it =smap.find(std::string("FMagnetic"));
    if (mg_it==smap.end()){
      cerr << "Attempting to load the new carmen log format" << endl;
      mg_it=smap.find(std::string("ROBOTMagnetic1"));
      assert(mg_it!=smap.end());
    }
    const MgSensor* mgSensor=dynamic_cast<const MgSensor*>((mg_it->second));

    m_matcher.setMgParameters(mgSensor->distance(), mgSensor->getPose());
  }
  
  void GridSlamProcessor::init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, OrientedPoint initialPose){
    m_xmin=xmin;
    m_ymin=ymin;
    m_xmax=xmax;
    m_ymax=ymax;
    m_delta=delta;
    if (m_infoStream)
      m_infoStream 
	<< " -xmin "<< m_xmin
	<< " -xmax "<< m_xmax
	<< " -ymin "<< m_ymin
	<< " -ymax "<< m_ymax
	<< " -delta "<< m_delta
	<< " -particles "<< size << endl;
    

    m_particles.clear();
    TNode* node=new TNode(initialPose, 0, 0, 0);
    ScanMatcherMap lmap(Point(xmin+xmax, ymin+ymax)*.5, xmax-xmin, ymax-ymin, delta);
    MgMatcherMap m_map(Point(xmin+xmax, ymin+ymax)*.5, xmax-xmin, ymax-ymin, 0.05);

    int x_num = 28;
    int y_num = 13;
    Vec3 mag_map[13][28];
    const float map_dist = 0.15;
    const float dist = 0.05;
    double map_width = round(x_num * map_dist * 10) / 10;
    double map_height = round(y_num * map_dist * 10) / 10;
    ifstream openFile("/home/d/catkin_ws/src/Gmapping_MagneticField/turtlebot_virtual_magnetic_sensor_simulator_metapkg/virtual_magnetic_sensor/map/magnetic_map.txt");
    if(openFile.is_open()){
      string line_str;
      int line = 0;
      while(getline(openFile, line_str)){
        std::stringstream ss(line_str);
        Vec3& cell = mag_map[line/x_num][line%x_num];
        ss >> cell.x;
        ss >> cell.y;
        ss >> cell.z;
        printf("Save map vec: %f %f %f to %d %d\n", cell.x, cell.y, cell.z, line/x_num, line%x_num);
        line++;
      }
      openFile.close();
    } 
    else{
      printf("cannot open mag map file...\n");
    }
    for(float i = 0; i<= (x_num-1)*map_dist; i+=dist)
      for(float j = 0; j<= (y_num-1)*map_dist; j+=dist){
        int v_i = (int)(i / map_dist);
        int v_j = (int)(j / map_dist);
        printf("v_i : %d v_j : %d, i : %f j : %f\n", v_i, v_j, i, j);
        Vec3& p1 = mag_map[v_j][v_i];
        Vec3& p2 = mag_map[v_j + 1][v_i];
        Vec3& p3 = mag_map[v_j][v_i + 1];
        Vec3& p4 = mag_map[v_j + 1][v_i + 1];
        
        Vec3 x1 = p1 + (p2 - p1) / map_dist * (i - map_dist * v_i);
        Vec3 x2 = p3 + (p4 - p3) / map_dist * (i - map_dist * v_i);
        Vec3 rslt = x1 + (x2 - x1) / map_dist * (j - map_dist * v_j);
        m_map.cell(Point(i - map_width/2, j - map_height/2)).update(true, rslt);
        if(m_map.cell(Point(i - map_width/2, j-map_height/2)).vec.x == 0) printf("fail cell %f\n");
        printf("Save map vec: %f %f %f using interpolation to %f, %f\n", rslt.x, rslt.y, rslt.z, i - map_width/2, j - map_height/2);
      }

    for (unsigned int i=0; i<size; i++){
      m_particles.push_back(Particle(lmap, m_map));
      m_particles.back().pose=initialPose;
      m_particles.back().previousPose=initialPose;
      m_particles.back().setWeight(0);
      m_particles.back().previousIndex=0;
      
		// this is not needed
		//		m_particles.back().node=new TNode(initialPose, 0, node, 0);

		// we use the root directly
		m_particles.back().node= node;
    }
    m_neff=(double)size;
    m_count=0;
    m_readingCount=0;
    m_linearDistance=m_angularDistance=0;
  }

  void GridSlamProcessor::processTruePos(const OdometryReading& o){
    const OdometrySensor* os=dynamic_cast<const OdometrySensor*>(o.getSensor());
    if (os && os->isIdeal() && m_outputStream){
      m_outputStream << setiosflags(ios::fixed) << setprecision(3);
      m_outputStream <<  "SIMULATOR_POS " <<  o.getPose().x << " " << o.getPose().y << " " ;
      m_outputStream << setiosflags(ios::fixed) << setprecision(6) << o.getPose().theta << " " <<  o.getTime() << endl;
    }
  }

  bool GridSlamProcessor::processScan_mg(const MgReading & reading, int adaptParticles){
    
    /**retireve the position from the reading, and compute the odometry*/
    OrientedPoint relPose=reading.getPose();  
    if (!m_count){
      m_lastPartPose=m_odoPose=relPose;
    }
    
    //write the state of the reading and update all the particles using the motion model

    // m_odoPose는 이전 위치, relpose는 현재 위치; 이 둘의 차이를 이용해 map에서의 파티클들의 위치를 부여,
    // 파티클들의 위치는 noise를 걸어서 각각 조금씩 다른 위치를 선택하게끔 한다.
    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      OrientedPoint& pose(it->pose);
      pose=m_motionModel.drawFromMotion(it->pose, relPose, m_odoPose);  //파티클들에게 위치 부여 
    }

    //printf("End Particle Posing\n");

    // update the output file
    if (m_outputStream.is_open()){
      m_outputStream << setiosflags(ios::fixed) << setprecision(6);
      m_outputStream << "ODOM ";
      m_outputStream << setiosflags(ios::fixed) << setprecision(3) << m_odoPose.x << " " << m_odoPose.y << " ";
      m_outputStream << setiosflags(ios::fixed) << setprecision(6) << m_odoPose.theta << " ";
      m_outputStream << reading.getTime();
      m_outputStream << endl;
    }
    if (m_outputStream.is_open()){
      m_outputStream << setiosflags(ios::fixed) << setprecision(6);
      m_outputStream << "ODO_UPDATE "<< m_particles.size() << " ";
      for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	OrientedPoint& pose(it->pose);
	m_outputStream << setiosflags(ios::fixed) << setprecision(3) << pose.x << " " << pose.y << " ";
	m_outputStream << setiosflags(ios::fixed) << setprecision(6) << pose.theta << " " << it-> weight << " ";
      }
      m_outputStream << reading.getTime();
      m_outputStream << endl;
    }
    
    //invoke the callback
    onOdometryUpdate();
    

    // accumulate the robot translation and rotation
    OrientedPoint move=relPose-m_odoPose;
    move.theta=atan2(sin(move.theta), cos(move.theta));
    m_linearDistance+=sqrt(move*move);
    m_angularDistance+=fabs(move.theta);

    //printf("linear : %f, angular : %f\n", m_linearDistance, m_angularDistance);
    
    // if the robot jumps throw a warning
    if (m_linearDistance>m_distanceThresholdCheck){
      cerr << "***********************************************************************" << endl;
      cerr << "********** Error: m_distanceThresholdCheck overridden!!!! *************" << endl;
      cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
      cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y 
	   << " " <<m_odoPose.theta << endl;
      cerr << "New Odometry Pose (reported from observation)= " << relPose.x << " " << relPose.y 
	   << " " <<relPose.theta << endl;
      cerr << "***********************************************************************" << endl;
      cerr << "** The Odometry has a big jump here. This is probably a bug in the   **" << endl;
      cerr << "** odometry/laser input. We continue now, but the result is probably **" << endl;
      cerr << "** crap or can lead to a core dump since the map doesn't fit.... C&G **" << endl;
      cerr << "***********************************************************************" << endl;
    }
    
    m_odoPose=relPose;
    
    bool processed=false;

    // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
    if (! m_count 
	|| m_linearDistance>=m_linearThresholdDistance 
	|| m_angularDistance>=m_angularThresholdDistance
    || (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_)){
      last_update_time_ = reading.getTime();      

      if (m_outputStream.is_open()){
	m_outputStream << setiosflags(ios::fixed) << setprecision(6);
	m_outputStream << "FRAME " <<  m_readingCount;
	m_outputStream << " " << m_linearDistance;
	m_outputStream << " " << m_angularDistance << endl;
      }
      
      if (m_infoStream)
	m_infoStream << "update frame " <<  m_readingCount << endl
		     << "update ld=" << m_linearDistance << " ad=" << m_angularDistance << endl;
      
      
      cerr << "Laser Pose= " << reading.getPose().x << " " << reading.getPose().y 
	   << " " << reading.getPose().theta << endl;
      
      
      //this is for converting the reading in a scan-matcher feedable form
      double (*plainReading)[3] = new double[5][3];
      for(unsigned int i=0; i<5; i++){
	plainReading[i][0]=reading[i].x;
	plainReading[i][1]=reading[i].y;
	plainReading[i][2]=reading[i].z;
      }
      m_infoStream << "m_count " << m_count << endl;

      MgReading* reading_copy = new MgReading(plainReading, static_cast<const MgSensor*>(reading.getSensor()));

      if (m_count>0){
        //얻어온 plainreading으로 파티클들의 가중치를 계산
	scanMatch_mg(plainReading);
	if (m_outputStream.is_open()){
	  OrientedPoint p=reading.getPose();
	  m_outputStream << setiosflags(ios::fixed) << setprecision(6);
	  m_outputStream << p.x << " " << p.y << " " << p.theta << " " << reading.getTime()<< endl;
	  m_outputStream << "SM_UPDATE "<< m_particles.size() << " ";
	  for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	    const OrientedPoint& pose=it->pose;
	    m_outputStream << setiosflags(ios::fixed) << setprecision(3) <<  pose.x << " " << pose.y << " ";
	    m_outputStream << setiosflags(ios::fixed) << setprecision(6) <<  pose.theta << " " << it-> weight << " ";
	  }
	  m_outputStream << endl;
	}
	onScanmatchUpdate();

  //파티클들에 연결된 Tnode에 가중치들을 normalize 하고 propagate
	updateTreeWeights(false);
				
	if (m_infoStream){
	  m_infoStream << "neff= " << m_neff  << endl;
	}
	if (m_outputStream.is_open()){
	  m_outputStream << setiosflags(ios::fixed) << setprecision(6);
	  m_outputStream << "NEFF " << m_neff << endl;
	}
 	resample_mg(plainReading, adaptParticles, reading_copy);
	
      } else {
	m_infoStream << "Registering First mg"<< endl;
	for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){	
    //각각 파티클에 map을 그림 
	  m_matcher.invalidateActiveArea();
	  m_matcher.computeActiveArea_mg(it->map_mg, it->pose, plainReading);
	  m_matcher.registerScan_mg(it->map_mg, it->pose, plainReading);
	  
	  // cyr: not needed anymore, particles refer to the root in the beginning!
    //현재 파티클의 위치와 센서값을 Tnode에 추가
	  TNode* node=new	TNode(it->pose, 0., it->node,  0);
    //node->reading=0;
      node->mg_reading = reading_copy;
      //node->reading = reading_copy;
	  it->node=node;
	  
	}
      }
      //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
      updateTreeWeights(false);
      //		cerr  << ".done!" <<endl;
      
      delete [] plainReading;
      m_lastPartPose=m_odoPose; //update the past pose for the next iteration
      m_linearDistance=0;
      m_angularDistance=0;
      m_count++;
      processed=true;
      
      //keep ready for the next step
      for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	it->previousPose=it->pose;
      }
      
    }
    if (m_outputStream.is_open())
      m_outputStream << flush;
    m_readingCount++;
    return processed;
  }
  
  
  bool GridSlamProcessor::processScan(const RangeReading & reading, int adaptParticles){
    
    /**retireve the position from the reading, and compute the odometry*/
    OrientedPoint relPose=reading.getPose();
    if (!m_count){
      m_lastPartPose=m_odoPose=relPose;
    }
    
    //write the state of the reading and update all the particles using the motion model
    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      OrientedPoint& pose(it->pose);
      pose=m_motionModel.drawFromMotion(it->pose, relPose, m_odoPose);
    }

    // update the output file
    if (m_outputStream.is_open()){
      m_outputStream << setiosflags(ios::fixed) << setprecision(6);
      m_outputStream << "ODOM ";
      m_outputStream << setiosflags(ios::fixed) << setprecision(3) << m_odoPose.x << " " << m_odoPose.y << " ";
      m_outputStream << setiosflags(ios::fixed) << setprecision(6) << m_odoPose.theta << " ";
      m_outputStream << reading.getTime();
      m_outputStream << endl;
    }
    if (m_outputStream.is_open()){
      m_outputStream << setiosflags(ios::fixed) << setprecision(6);
      m_outputStream << "ODO_UPDATE "<< m_particles.size() << " ";
      for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	OrientedPoint& pose(it->pose);
	m_outputStream << setiosflags(ios::fixed) << setprecision(3) << pose.x << " " << pose.y << " ";
	m_outputStream << setiosflags(ios::fixed) << setprecision(6) << pose.theta << " " << it-> weight << " ";
      }
      m_outputStream << reading.getTime();
      m_outputStream << endl;
    }
    
    //invoke the callback
    onOdometryUpdate();
    

    // accumulate the robot translation and rotation
    OrientedPoint move=relPose-m_odoPose;
    move.theta=atan2(sin(move.theta), cos(move.theta));
    m_linearDistance+=sqrt(move*move);
    m_angularDistance+=fabs(move.theta);
    
    // if the robot jumps throw a warning
    if (m_linearDistance>m_distanceThresholdCheck){
      cerr << "***********************************************************************" << endl;
      cerr << "********** Error: m_distanceThresholdCheck overridden!!!! *************" << endl;
      cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
      cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y 
	   << " " <<m_odoPose.theta << endl;
      cerr << "New Odometry Pose (reported from observation)= " << relPose.x << " " << relPose.y 
	   << " " <<relPose.theta << endl;
      cerr << "***********************************************************************" << endl;
      cerr << "** The Odometry has a big jump here. This is probably a bug in the   **" << endl;
      cerr << "** odometry/laser input. We continue now, but the result is probably **" << endl;
      cerr << "** crap or can lead to a core dump since the map doesn't fit.... C&G **" << endl;
      cerr << "***********************************************************************" << endl;
    }
    
    m_odoPose=relPose;
    
    bool processed=false;

    // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
    if (! m_count 
	|| m_linearDistance>=m_linearThresholdDistance 
	|| m_angularDistance>=m_angularThresholdDistance
    || (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_)){
      last_update_time_ = reading.getTime();      

      if (m_outputStream.is_open()){
	m_outputStream << setiosflags(ios::fixed) << setprecision(6);
	m_outputStream << "FRAME " <<  m_readingCount;
	m_outputStream << " " << m_linearDistance;
	m_outputStream << " " << m_angularDistance << endl;
      }
      
      if (m_infoStream)
	m_infoStream << "update frame " <<  m_readingCount << endl
		     << "update ld=" << m_linearDistance << " ad=" << m_angularDistance << endl;
      
      
      cerr << "Laser Pose= " << reading.getPose().x << " " << reading.getPose().y 
	   << " " << reading.getPose().theta << endl;
      
      
      //this is for converting the reading in a scan-matcher feedable form
      assert(reading.size()==m_beams);
      double * plainReading = new double[m_beams];
      for(unsigned int i=0; i<m_beams; i++){
	plainReading[i]=reading[i];
      }
      m_infoStream << "m_count " << m_count << endl;

      RangeReading* reading_copy = 
              new RangeReading(reading.size(),
                               &(reading[0]),
                               static_cast<const RangeSensor*>(reading.getSensor()),
                               reading.getTime());

      if (m_count>0){
	scanMatch(plainReading);
	if (m_outputStream.is_open()){
	  m_outputStream << "LASER_READING "<< reading.size() << " ";
	  m_outputStream << setiosflags(ios::fixed) << setprecision(2);
	  for (RangeReading::const_iterator b=reading.begin(); b!=reading.end(); b++){
	    m_outputStream << *b << " ";
	  }
	  OrientedPoint p=reading.getPose();
	  m_outputStream << setiosflags(ios::fixed) << setprecision(6);
	  m_outputStream << p.x << " " << p.y << " " << p.theta << " " << reading.getTime()<< endl;
	  m_outputStream << "SM_UPDATE "<< m_particles.size() << " ";
	  for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	    const OrientedPoint& pose=it->pose;
	    m_outputStream << setiosflags(ios::fixed) << setprecision(3) <<  pose.x << " " << pose.y << " ";
	    m_outputStream << setiosflags(ios::fixed) << setprecision(6) <<  pose.theta << " " << it-> weight << " ";
	  }
	  m_outputStream << endl;
	}
	onScanmatchUpdate();
	
	updateTreeWeights(false);
				
	if (m_infoStream){
	  m_infoStream << "neff= " << m_neff  << endl;
	}
	if (m_outputStream.is_open()){
	  m_outputStream << setiosflags(ios::fixed) << setprecision(6);
	  m_outputStream << "NEFF " << m_neff << endl;
	}
 	resample(plainReading, adaptParticles, reading_copy);
	
      } else {
	m_infoStream << "Registering First Scan"<< endl;
	for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){	
	  m_matcher.invalidateActiveArea();
	  m_matcher.computeActiveArea(it->map, it->pose, plainReading);
	  m_matcher.registerScan(it->map, it->pose, plainReading);
	  
	  // cyr: not needed anymore, particles refer to the root in the beginning!
	  TNode* node=new	TNode(it->pose, 0., it->node,  0);
	  //node->reading=0;
      node->reading = reading_copy;
	  it->node=node;
	  
	}
      }
      //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
      updateTreeWeights(false);
      //		cerr  << ".done!" <<endl;
      
      delete [] plainReading;
      m_lastPartPose=m_odoPose; //update the past pose for the next iteration
      m_linearDistance=0;
      m_angularDistance=0;
      m_count++;
      processed=true;
      
      //keep ready for the next step
      for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	it->previousPose=it->pose;
      }
      
    }
    if (m_outputStream.is_open())
      m_outputStream << flush;
    m_readingCount++;
    return processed;
  }
  
  
  std::ofstream& GridSlamProcessor::outputStream(){
    return m_outputStream;
  }
  
  std::ostream& GridSlamProcessor::infoStream(){
    return m_infoStream;
  }
  
  
  int GridSlamProcessor::getBestParticleIndex() const{
    unsigned int bi=0;
    double bw=-std::numeric_limits<double>::max();
    for (unsigned int i=0; i<m_particles.size(); i++)
      if (bw<m_particles[i].weightSum){
	bw=m_particles[i].weightSum;
	bi=i;
      }
    return (int) bi;
  }

  void GridSlamProcessor::onScanmatchUpdate(){}
  void GridSlamProcessor::onResampleUpdate(){}
  void GridSlamProcessor::onOdometryUpdate(){}

  
};// end namespace




