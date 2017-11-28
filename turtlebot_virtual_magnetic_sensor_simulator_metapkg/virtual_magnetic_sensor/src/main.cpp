
#include <stdio.h>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <magnetic_sensor/MagneticFields.h>
#include <visualization_msgs/Marker.h>
#include <virtual_magnetic_sensor/req_rviz_marker.h>

#include <tf/transform_broadcaster.h>
#include <iostream>

#include <semaphore.h>
#include <fstream>


//ROS
ros::NodeHandle *nh;
ros::Publisher pub_marker;
ros::ServiceServer server;


//semaphore
sem_t sem_kobuki_actual_pose;


//magnetic field map
tf::Vector3 **mag_map;//28*13 = 364
int x_num = 28;
int y_num = 13;
double grid_x_len = 0.2;//20cm
double grid_y_len = 0.15;//15cm
double map_width;//meter
double map_height;//meter
int reverse_x_axis_dir = 0;//1 is reverse activate
int reverse_y_axis_dir = 0;//1 is reverse activate


//kobuki pose
tf::Vector3 kobuki_actual_pos;
tf::Quaternion kobuki_actual_ori;
//double kobuki_actual_rpy[3];


//magnetic sensor pose
tf::Transform mag_transform_rotate;//base_link to magnetic sensor
tf::Vector3 mag_delta_pos[5];//pos from baselink
tf::Vector3 mag_actual_pos[5];
tf::Quaternion mag_actual_ori;//all magnetic sensor same value
int mag_pose_is_valid = 0;



void calc_magnetic_sensor_pose(void)
{
	sem_wait(&sem_kobuki_actual_pose);

	mag_actual_pos[0] = kobuki_actual_pos;
	mag_actual_pos[1] = kobuki_actual_pos;
	mag_actual_pos[2] = kobuki_actual_pos;
	mag_actual_pos[3] = kobuki_actual_pos;
	mag_actual_pos[4] = kobuki_actual_pos;

	mag_actual_ori = kobuki_actual_ori;

	sem_post(&sem_kobuki_actual_pose);

	mag_transform_rotate.setRotation(mag_actual_ori);

	mag_actual_pos[0] += mag_transform_rotate * mag_delta_pos[0];
	mag_actual_pos[1] += mag_transform_rotate * mag_delta_pos[1];
	mag_actual_pos[2] += mag_transform_rotate * mag_delta_pos[2];
	mag_actual_pos[3] += mag_transform_rotate * mag_delta_pos[3];
	mag_actual_pos[4] += mag_transform_rotate * mag_delta_pos[4];
}


int get_world_magnetic_field_vector(tf::Vector3 world_pos, tf::Vector3& world_mag_vector)
{
	tf::Vector3 x_interp1, x_interp2;

	world_pos[0] += map_width/2;//to move center
	world_pos[1] += map_height/2;//to move center

	if(reverse_x_axis_dir == 1)
		world_pos[0] = map_width - world_pos[0];
	if(reverse_y_axis_dir == 1)
		world_pos[1] = map_height - world_pos[1];

	int x_index;
	int y_index;
	x_index = (int)(world_pos[0] / grid_x_len);
	y_index = (int)(world_pos[1] / grid_y_len);
	//printf("index : %d %d\n", x_index, y_index);

	if(x_index < 0 || y_index < 0 || x_index > x_num - 2 || y_index > y_num - 2)
		return -1;//sensor is out of map

	double x_offset;
	double y_offset;
	x_offset = world_pos[0] - grid_x_len * x_index;
	y_offset = world_pos[1] - grid_y_len * y_index;

	x_interp1[0] = mag_map[y_index][x_index][0] + ((mag_map[y_index][x_index+1][0] - mag_map[y_index][x_index][0]) * (x_offset/grid_x_len));
	x_interp1[1] = mag_map[y_index][x_index][1] + ((mag_map[y_index][x_index+1][1] - mag_map[y_index][x_index][1]) * (x_offset/grid_x_len));
	x_interp1[2] = mag_map[y_index][x_index][2] + ((mag_map[y_index][x_index+1][2] - mag_map[y_index][x_index][2]) * (x_offset/grid_x_len));

	x_interp2[0] = mag_map[y_index+1][x_index][0] + ((mag_map[y_index+1][x_index+1][0] - mag_map[y_index+1][x_index][0]) * (x_offset/grid_x_len));
	x_interp2[1] = mag_map[y_index+1][x_index][1] + ((mag_map[y_index+1][x_index+1][1] - mag_map[y_index+1][x_index][1]) * (x_offset/grid_x_len));
	x_interp2[2] = mag_map[y_index+1][x_index][2] + ((mag_map[y_index+1][x_index+1][2] - mag_map[y_index+1][x_index][2]) * (x_offset/grid_x_len));

	world_mag_vector[0] = x_interp1[0] + ((x_interp2[0] - x_interp1[0]) * (y_offset/grid_y_len));
	world_mag_vector[1] = x_interp1[1] + ((x_interp2[1] - x_interp1[1]) * (y_offset/grid_y_len));
	world_mag_vector[2] = x_interp1[2] + ((x_interp2[2] - x_interp1[2]) * (y_offset/grid_y_len));

	return 0;
}


int get_mag_sensor_value_from_world_mag_vector(tf::Vector3 world_mag_vector, tf::Quaternion mag_sensor_ori, tf::Vector3& mag_sensor_value)
{
	tf::Transform mag_transform_rotate;

	mag_transform_rotate.setRotation(mag_sensor_ori);
	
	mag_sensor_value = mag_transform_rotate.inverse() * world_mag_vector;

	return 0;
}


int marker_publish_enable[6];
bool srv_callback(virtual_magnetic_sensor::req_rviz_marker::Request &req, virtual_magnetic_sensor::req_rviz_marker::Response &res)
{
	printf("service callback req : %d, %d\n", req.req1, req.req2);

	res.result = 1;

	switch(req.req1)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			if(req.req2 < 0 || req.req2 > 1)
				res.result = -1;
			else
				marker_publish_enable[req.req1] = req.req2;

			break;

		case 5:
			if(req.req2 < 0 || req.req2 > 1)
				res.result = -1;
			else
			{
				marker_publish_enable[0] = req.req2;
				marker_publish_enable[1] = req.req2;
				marker_publish_enable[2] = req.req2;
				marker_publish_enable[3] = req.req2;
				marker_publish_enable[4] = req.req2;
			}

			break;

		case 10:
			if(req.req2 < 0 || req.req2 > 4)
				res.result = -1;
			else
				marker_publish_enable[5] = req.req2;

			break;
		default:
			res.result = -1;
	}
}


visualization_msgs::Marker mag_sensor_marker[5];
visualization_msgs::Marker mag_map_marker;
int marker_id = 0;
void init_rviz_marker(void)//for debug
{
	pub_marker = nh -> advertise<visualization_msgs::Marker>("mag_vector", 500, 0);
	server = nh -> advertiseService("marker_service", srv_callback);

	for(int i = 0 ; i < 5 ; i++)
	{
		marker_publish_enable[i] = 0;

		mag_sensor_marker[i].header.frame_id = "gazebo_world";
		mag_sensor_marker[i].ns = "my_namespace";
		mag_sensor_marker[i].action = visualization_msgs::Marker::ADD;
		mag_sensor_marker[i].type = 0;

		mag_sensor_marker[i].scale.x = 0.005;
		mag_sensor_marker[i].scale.y = 0.01;
		mag_sensor_marker[i].scale.z = 0.02;

		mag_sensor_marker[i].color.a = 1.0;//alpha
	}

	mag_sensor_marker[0].color.r = 1.0;
	mag_sensor_marker[0].color.g = 0.0;
	mag_sensor_marker[0].color.b = 0.0;

	mag_sensor_marker[1].color.r = 0.0;
	mag_sensor_marker[1].color.g = 1.0;
	mag_sensor_marker[1].color.b = 0.0;
	
	mag_sensor_marker[2].color.r = 0.0;
	mag_sensor_marker[2].color.g = 0.0;
	mag_sensor_marker[2].color.b = 1.0;

	mag_sensor_marker[3].color.r = 1.0;
	mag_sensor_marker[3].color.g = 1.0;
	mag_sensor_marker[3].color.b = 0.0;
	
	mag_sensor_marker[4].color.r = 1.0;
	mag_sensor_marker[4].color.g = 1.0;
	mag_sensor_marker[4].color.b = 1.0;


	marker_publish_enable[5] = 0;

	mag_map_marker.header.frame_id = "gazebo_world";
	mag_map_marker.ns = "my_namespace";
	mag_map_marker.action = visualization_msgs::Marker::ADD;
	mag_map_marker.type = 0;

	mag_map_marker.scale.x = 0.005;
	mag_map_marker.scale.y = 0.01;
	mag_map_marker.scale.z = 0.02;

	mag_map_marker.color.a = 1.0;//alpha

	mag_map_marker.color.r = 1.0;
	mag_map_marker.color.g = 0.0;
	mag_map_marker.color.b = 0.0;
}


void publish_rviz_marker(int marker_index, geometry_msgs::Point start_point, geometry_msgs::Point end_point)//for debug, publish marker for rviz
{
	//marker_index => 0~4 : mag sensor marker, 5 : mag map marker

	switch(marker_index)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			mag_sensor_marker[marker_index].header.stamp = ros::Time();
			mag_sensor_marker[marker_index].id = marker_id++;//overflow possible

			mag_sensor_marker[marker_index].points.clear();
			mag_sensor_marker[marker_index].points.push_back(start_point);
			mag_sensor_marker[marker_index].points.push_back(end_point);

			pub_marker.publish(mag_sensor_marker[marker_index]);

			break;

		case 5:
			mag_map_marker.header.stamp = ros::Time();
			mag_map_marker.id = marker_id++;//overflow possible

			mag_map_marker.points.clear();
			mag_map_marker.points.push_back(start_point);
			mag_map_marker.points.push_back(end_point);

			pub_marker.publish(mag_map_marker);

			break;
	}
}


int gzb_msg_last_index = 0;
void callback(const gazebo_msgs::ModelStates &msg)
{
//	printf("callback\n");

	for(int i = gzb_msg_last_index ; i < msg.name.size() ; i++)
	{
		if(msg.name[i].compare("mobile_base") == 0)
		{
			sem_wait(&sem_kobuki_actual_pose);

			kobuki_actual_pos[0] = msg.pose[i].position.x;
			kobuki_actual_pos[1] = msg.pose[i].position.y;
			kobuki_actual_pos[2] = msg.pose[i].position.z;

			kobuki_actual_ori[0] = msg.pose[i].orientation.x;
			kobuki_actual_ori[1] = msg.pose[i].orientation.y;
			kobuki_actual_ori[2] = msg.pose[i].orientation.z;
			kobuki_actual_ori[3] = msg.pose[i].orientation.w;

			sem_post(&sem_kobuki_actual_pose);

			gzb_msg_last_index = i;
			mag_pose_is_valid = 1;

			return;
		}
	}

	gzb_msg_last_index = 0;//toss next callback
}


int main(int argc, char** argv)
{
	if(sem_init(&sem_kobuki_actual_pose, 0, 1) == -1)
	{
		printf("sem_init error\n");
		exit(0);
	}

	ros::init(argc, argv, "virtual_magnetic_sensor");
	nh = new ros::NodeHandle();

	int sensor_sampling_rate;
	std::string filePath;

	if(nh -> getParam("magnetic_sensor_tf/mag1_x", mag_delta_pos[0][0]) == 0)
	{
		printf("WARNING!! CANNOT GET MAGNETIC SENSOR POSITION FROM PARAMETER SERVER!!\n");
	}
	nh -> getParam("magnetic_sensor_tf/mag1_y", mag_delta_pos[0][1]);
	nh -> getParam("magnetic_sensor_tf/mag1_z", mag_delta_pos[0][2]);
	nh -> getParam("magnetic_sensor_tf/mag2_x", mag_delta_pos[1][0]);
	nh -> getParam("magnetic_sensor_tf/mag2_y", mag_delta_pos[1][1]);
	nh -> getParam("magnetic_sensor_tf/mag2_z", mag_delta_pos[1][2]);
	nh -> getParam("magnetic_sensor_tf/mag3_x", mag_delta_pos[2][0]);
	nh -> getParam("magnetic_sensor_tf/mag3_y", mag_delta_pos[2][1]);
	nh -> getParam("magnetic_sensor_tf/mag3_z", mag_delta_pos[2][2]);
	nh -> getParam("magnetic_sensor_tf/mag4_x", mag_delta_pos[3][0]);
	nh -> getParam("magnetic_sensor_tf/mag4_y", mag_delta_pos[3][1]);
	nh -> getParam("magnetic_sensor_tf/mag4_z", mag_delta_pos[3][2]);
	nh -> getParam("magnetic_sensor_tf/mag5_x", mag_delta_pos[4][0]);
	nh -> getParam("magnetic_sensor_tf/mag5_y", mag_delta_pos[4][1]);
	nh -> getParam("magnetic_sensor_tf/mag5_z", mag_delta_pos[4][2]);

	nh -> getParam("virtual_magnetic_sensor/sampling_rate", sensor_sampling_rate);
	nh -> getParam("virtual_magnetic_sensor/map_file", filePath);
	nh -> getParam("virtual_magnetic_sensor/map_x_num", x_num);
	nh -> getParam("virtual_magnetic_sensor/map_y_num", y_num);
	nh -> getParam("virtual_magnetic_sensor/map_grid_x_len", grid_x_len);
	nh -> getParam("virtual_magnetic_sensor/map_grid_y_len", grid_y_len);
	nh -> getParam("virtual_magnetic_sensor/reverse_x_axis_dir", reverse_x_axis_dir);
	nh -> getParam("virtual_magnetic_sensor/reverse_y_axis_dir", reverse_y_axis_dir);

	printf("sensor sampling rate : %d\n", sensor_sampling_rate);
	printf("input map file : %s\n", filePath.c_str());

	mag_transform_rotate.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

	mag_map = new tf::Vector3*[y_num];
	for(int i = 0 ; i < y_num ; i++)
		mag_map[i] = new tf::Vector3[x_num];

	map_width = x_num * grid_x_len;
	map_height = y_num * grid_y_len;

	std::ifstream openFile(filePath.data());
	if(openFile.is_open())
	{
		std::string line_str;
		int line = 0;
		while(getline(openFile, line_str))
		{
			std::stringstream ss(line_str);
			ss >> mag_map[line/x_num][line%x_num][0];
			ss >> mag_map[line/x_num][line%x_num][1];
			ss >> mag_map[line/x_num][line%x_num][2];
			line++;
		}

		openFile.close();
	}
	else
	{
		printf("cannot open magnetic map file...\n");

		exit(0);
	}

	for(int y = 0 ; y < y_num ; y++)
	{
		for(int x = 0 ; x < x_num ; x++)
		{
			//std::cout << "x: " << mag_map[y][x][0] << ", y: " << mag_map[y][x][1] << ",z: "  << mag_map[y][x][2] << std::endl;
		}
	}

	ros::Subscriber sub = nh -> subscribe("/gazebo/model_states", 1, callback);
	ros::Publisher pub = nh -> advertise<magnetic_sensor::MagneticFields>("mag_scan", 1, 0);// topic name, queue size, latch

	ros::AsyncSpinner spinner(1);//Use 1 threads
	spinner.start();

	int wait_cnt = 0;
	while(mag_pose_is_valid == 0)
	{
		usleep(100000);

		if(wait_cnt++ > 50)
		{
			wait_cnt = 0;

			printf("waiting for gazebo topic...\n");
		}
	}


	ros::Rate hz(sensor_sampling_rate);

	tf::Vector3 world_mag_vector[5];
	tf::Vector3 mag_sensor_output_data[5];
	geometry_msgs::Point start_point;
	geometry_msgs::Point end_point;

	magnetic_sensor::MagneticFields msg;
	msg.header.frame_id = "mag5";

	init_rviz_marker();//for debug with rviz Marker

	while(ros::ok())
	{
		calc_magnetic_sensor_pose();

		int result = 0;
		if(get_world_magnetic_field_vector(mag_actual_pos[0], world_mag_vector[0]) < 0)
			result = -1;
		if(get_world_magnetic_field_vector(mag_actual_pos[1], world_mag_vector[1]) < 0)
			result = -1;
		if(get_world_magnetic_field_vector(mag_actual_pos[2], world_mag_vector[2]) < 0)
			result = -1;
		if(get_world_magnetic_field_vector(mag_actual_pos[3], world_mag_vector[3]) < 0)
			result = -1;
		if(get_world_magnetic_field_vector(mag_actual_pos[4], world_mag_vector[4]) < 0)
			result = -1;

		if(result == -1)
		{
			printf("robot is out of the magnetic field map!\n");

			hz.sleep();

			continue;
		}

		get_mag_sensor_value_from_world_mag_vector(world_mag_vector[0], mag_actual_ori, mag_sensor_output_data[0]);
		get_mag_sensor_value_from_world_mag_vector(world_mag_vector[1], mag_actual_ori, mag_sensor_output_data[1]);
		get_mag_sensor_value_from_world_mag_vector(world_mag_vector[2], mag_actual_ori, mag_sensor_output_data[2]);
		get_mag_sensor_value_from_world_mag_vector(world_mag_vector[3], mag_actual_ori, mag_sensor_output_data[3]);
		get_mag_sensor_value_from_world_mag_vector(world_mag_vector[4], mag_actual_ori, mag_sensor_output_data[4]);

		msg.header.stamp = ros::Time::now();

		printf("...\n");
		printf("mags actual ori: %lf, %lf, %lf, %lf\n", mag_actual_ori[0], mag_actual_ori[1], mag_actual_ori[2], mag_actual_ori[3]);

		for(int sensor_index = 0 ; sensor_index < 5 ; sensor_index++)
		{
				msg.magnetic_field[sensor_index].x = mag_sensor_output_data[sensor_index][0];
				msg.magnetic_field[sensor_index].y = mag_sensor_output_data[sensor_index][1];
				msg.magnetic_field[sensor_index].z = mag_sensor_output_data[sensor_index][2];

				printf("mag%d actual pos: %lf, %lf, %lf\n", sensor_index, mag_actual_pos[sensor_index][0], mag_actual_pos[sensor_index][1], mag_actual_pos[sensor_index][2]);
				printf("mag%d value : %lf %lf %lf\n", sensor_index, msg.magnetic_field[sensor_index].x, msg.magnetic_field[sensor_index].y, msg.magnetic_field[sensor_index].z);	
		}

		pub.publish(msg);


		// vvv for debug with rviz Marker vvv
		tf::Vector3 marker_pos;
		tf::Vector3 world_mag_vector_marker;
		for(int i = 0 ; i < 6 ; i++)
		{	
			if(marker_publish_enable[i] == 0)
				continue;

			if(i < 5)
			{
				start_point.x = mag_actual_pos[i][0];
				start_point.y = mag_actual_pos[i][1];
				start_point.z = mag_actual_pos[i][2];
				end_point.x = mag_actual_pos[i][0] + world_mag_vector[i][0] * 0.0001;//0.0001 is arrow length gain
				end_point.y = mag_actual_pos[i][1] + world_mag_vector[i][1] * 0.0001;
				end_point.z = mag_actual_pos[i][2] + world_mag_vector[i][2] * 0.0001;

				publish_rviz_marker(i, start_point, end_point);
			}
			else
			{
				// OVERHEAD!!
				marker_pos[2] = mag_actual_pos[0][2];//just get height
				for(int y = 0 ; y < y_num ; y++)
				{
					marker_pos[1] = y * grid_y_len - map_height/2;
					for(int x = 0 ; x < x_num ; x++)
					{
						marker_pos[0] = x * grid_x_len - map_width/2;

						if(get_world_magnetic_field_vector(marker_pos, world_mag_vector_marker) < 0)
							continue;

						if(marker_publish_enable[i] > 1)//get 1 axis map
						{
							start_point.x = marker_pos[0];
							start_point.y = marker_pos[1];
							start_point.z = marker_pos[2] + world_mag_vector_marker[marker_publish_enable[i] - 2] * 0.001;
							end_point.x = marker_pos[0] + world_mag_vector_marker[0] * 0.0001;//0.0001 is arrow length gain
							end_point.y = marker_pos[1] + world_mag_vector_marker[1] * 0.0001;
							end_point.z = marker_pos[2] + world_mag_vector_marker[marker_publish_enable[i] - 2] * 0.001 + world_mag_vector_marker[2] * 0.0001;
						}
						else
						{
							start_point.x = marker_pos[0];
							start_point.y = marker_pos[1];
							start_point.z = marker_pos[2];
							end_point.x = marker_pos[0] + world_mag_vector_marker[0] * 0.0001;//0.0001 is arrow length gain
							end_point.y = marker_pos[1] + world_mag_vector_marker[1] * 0.0001;
							end_point.z = marker_pos[2] + world_mag_vector_marker[2] * 0.0001;
						}

						publish_rviz_marker(i, start_point, end_point);
					}
				}

				marker_publish_enable[i] = 0;
			}

		}
		// ^^^ for debug with rviz Marker ^^^

		hz.sleep();
	}

	return 0;
}
