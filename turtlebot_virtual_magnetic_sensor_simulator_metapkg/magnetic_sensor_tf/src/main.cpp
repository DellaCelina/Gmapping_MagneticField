#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "magnetic_sensor_tf");
	ros::NodeHandle nh;

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;

//	std::string mag_pos_str[5][3];
	double mag_pos[5][3];

	nh.getParam("magnetic_sensor_tf/mag1_x", mag_pos[0][0]);
	nh.getParam("magnetic_sensor_tf/mag1_y", mag_pos[0][1]);
	nh.getParam("magnetic_sensor_tf/mag1_z", mag_pos[0][2]);
	nh.getParam("magnetic_sensor_tf/mag2_x", mag_pos[1][0]);
	nh.getParam("magnetic_sensor_tf/mag2_y", mag_pos[1][1]);
	nh.getParam("magnetic_sensor_tf/mag2_z", mag_pos[1][2]);
	nh.getParam("magnetic_sensor_tf/mag3_x", mag_pos[2][0]);
	nh.getParam("magnetic_sensor_tf/mag3_y", mag_pos[2][1]);
	nh.getParam("magnetic_sensor_tf/mag3_z", mag_pos[2][2]);
	nh.getParam("magnetic_sensor_tf/mag4_x", mag_pos[3][0]);
	nh.getParam("magnetic_sensor_tf/mag4_y", mag_pos[3][1]);
	nh.getParam("magnetic_sensor_tf/mag4_z", mag_pos[3][2]);
	nh.getParam("magnetic_sensor_tf/mag5_x", mag_pos[4][0]);
	nh.getParam("magnetic_sensor_tf/mag5_y", mag_pos[4][1]);
	nh.getParam("magnetic_sensor_tf/mag5_z", mag_pos[4][2]);

	while(nh.ok())
	{
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(
					tf::Quaternion(0, 0, 0, 1),
					tf::Vector3(mag_pos[0][0], mag_pos[0][1], mag_pos[0][2])
				),
				ros::Time::now(),
				"base_link",
				"mag1"
			)
		);

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(
					tf::Quaternion(0, 0, 0, 1),
					tf::Vector3(mag_pos[1][0], mag_pos[1][1], mag_pos[1][2])
				),
				ros::Time::now(),
				"base_link",
				"mag2"
			)
		);

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(
					tf::Quaternion(0, 0, 0, 1),
					tf::Vector3(mag_pos[2][0], mag_pos[2][1], mag_pos[2][2])
				),
				ros::Time::now(),
				"base_link",
				"mag3"
			)
		);

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(
					tf::Quaternion(0, 0, 0, 1),
					tf::Vector3(mag_pos[3][0], mag_pos[3][1], mag_pos[3][2])
				),
				ros::Time::now(),
				"base_link",
				"mag4"
			)
		);

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(
					tf::Quaternion(0, 0, 0, 1),
					tf::Vector3(mag_pos[4][0], mag_pos[4][1], mag_pos[4][2])
				),
				ros::Time::now(),
				"base_link",
				"mag5"
			)
		);

		r.sleep();
	}
}


