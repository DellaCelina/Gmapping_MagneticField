#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>


tf::TransformBroadcaster *broadcaster;


int gzb_msg_last_index = 0;
void callback(const gazebo_msgs::ModelStates &msg)
{
//	printf("callback\n");

	for(int i = gzb_msg_last_index ; i < msg.name.size() ; i++)
	{
		if(msg.name[i].compare("mobile_base") == 0)
		{
			broadcaster -> sendTransform(
				tf::StampedTransform(
					tf::Transform(
						tf::Quaternion(msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w),
						tf::Vector3(msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z)
					).inverse(),
					ros::Time::now(),
					"base_footprint",
					"gazebo_world"
				)
			);

			gzb_msg_last_index = i;

			return;
		}
	}

	gzb_msg_last_index = 0;//toss next callback
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "gazebo_tf");
	ros::NodeHandle nh;

	broadcaster = new tf::TransformBroadcaster();
	ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1, callback);

	ros::spin();

	ros::Rate r(1);

	while(nh.ok())
	{
		r.sleep();
	}
}


