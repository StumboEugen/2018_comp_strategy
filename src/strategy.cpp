float SPD_MAX = 0.5;
float SAFE_RANGE = 0.8;
float STOP_RANGE = 0.2;
float MAP_ORIGIN_TO_STRUCT_X = 1.0;
float MAP_ORIGIN_TO_STRUCT_Y = 1.0;
float SCAN_DELAY = 2.0;
float SCAN_SPEED = 1.0;
float BRAKE_COE = 1.0;
float HEIGHT = 1.0;
float SPD_CLIMB = 0.1;
float SPD_TURN = 0.5;
float ANGLE_TOLERANCE = 0.3;
int WAYPOINT_GROUP = 1;
bool OUTPUT_GAZEBO = true;
bool OUTPUT_PX4 = true;

#include "ros/ros.h"

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CameraPose.h>
#include <px4_autonomy/Takeoff.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Position.h>

#include <sstream>

ros::Publisher pub_spd_gazebo;
ros::Publisher pub_spd_px4;
ros::Publisher pub_cam;
ros::Publisher pub_pose_px4;

void sendSpd_px4(float vx, float vy, float vz, float wz) {
	px4_autonomy::Velocity spd;
	spd.x = -vy;
	spd.y = vx;
	spd.z = vz;
	spd.yaw_rate = wz;
	spd.header.stamp = ros::Time::now();
	pub_spd_px4.publish(spd);
}

void sendPose_px4 (float px, float py, float pz, float yaw) {
	px4_autonomy::Position pos;
	pos.x = px;
	pos.y = py;
	pos.z = pz;
	pos.yaw = yaw;
	pos.header.stamp = ros::Time::now();
	pub_pose_px4.publish(pos);
}

float current_theta; //from laserCB
void sendSpd_gazebo(float vx, float vy, float wz) {
	ROS_DEBUG("map vel vx vy:%.3f\t%.3f", vx, vy);
	geometry_msgs::Twist spd;
	spd.linear.x = vx * cos(-current_theta) - vy * sin(-current_theta);
	spd.linear.y = vx * sin(-current_theta) + vy * cos(-current_theta);
	spd.angular.z = wz;
	pub_spd_gazebo.publish(spd);
}


void sendSpd(float vx, float vy, float wz) {
	if (OUTPUT_GAZEBO) {
		sendSpd_gazebo(vx,vy,wz);
	}

	if (OUTPUT_PX4) {
		sendSpd_px4(vx,vy,0,wz);
	}
}

void sendSpd(float vx, float vy) {
	sendSpd(vx, vy, 0);
}

void stop() {
	sendSpd(0,0,0);
}

/////////////////////////////////////////////

typedef struct _pos_points_s
{
	float x;
	float y;
	bool need_scan;
	float waypoint_delay;
	bool set_ori;
	float ori;
} _pos_points_t;

int progress = 0;	//the current target to go
int numofWaypoints = 0;
_pos_points_t wayPoints[50];

void addPosPoint(float x, float y, bool need_scan, float delaytime) {
	_pos_points_t pos;
	pos.x = x + MAP_ORIGIN_TO_STRUCT_X;
	pos.y = y + MAP_ORIGIN_TO_STRUCT_Y;
	pos.need_scan = need_scan;
	pos.waypoint_delay = delaytime;
	pos.set_ori = false;
	wayPoints[numofWaypoints] = pos;
	numofWaypoints ++;
}

void addPosPoint(float x, float y, bool need_scan) {
	addPosPoint(x, y, need_scan, 0);
}

void addPosPoint(float x, float y, float delaytime) {
	addPosPoint(x, y, false, delaytime);
}

void addPosPoint(float x, float y) {
	addPosPoint(x, y, false);
}

void setOriForLastPoint(float ori) {
	wayPoints[numofWaypoints - 1].set_ori = true;
	wayPoints[numofWaypoints - 1].ori = ori;
}

bool wayPointInit();

///////////////////////////////////////////

float current_x;
float current_y;

bool gotlaser = false;

void laserCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	gotlaser = true;
	current_x = msg -> pose.pose.position.x;
	current_y = msg -> pose.pose.position.y;
	current_theta = asin(msg -> pose.pose.orientation.z) * 2;
	// ROS_INFO("theta now: %.3f", current_theta);
}

void scan() {
	ROS_INFO("start scan");
	static bool last_scan_ori;
	mavros_msgs::CameraPose cam_msg;
	cam_msg.yaw = last_scan_ori ? 1 : -1;
	cam_msg.yaw *= SCAN_SPEED;
	last_scan_ori = !last_scan_ori;
	pub_cam.publish(cam_msg);
	ros::Duration(SCAN_DELAY).sleep();
	cam_msg.yaw = 0;
	pub_cam.publish(cam_msg);
	ROS_INFO("scan done");
}

int status;

void statusCB(const std_msgs::UInt8::ConstPtr& msg) {
	status = msg -> data;
}

float pose_x, pose_y, pose_z, pose_yaw;
px4_autonomy::Position pose_current;
void poseCB(const px4_autonomy::Position &msg) {
	pose_x = msg.x;
	pose_y = msg.y;
	pose_z = msg.z;
	pose_yaw = msg.yaw;
	pose_current = msg;
}

float set_point_z;
void setPointCB(const geometry_msgs::PoseStamped &msg) {
	set_point_z = msg.pose.position.z;
}

ros::Publisher pub_takeoff;

void land() {
	while (status != 4 && status != 5) {
		ros::Duration(1).sleep();
		ROS_INFO("status not 4 or 5!!");
		ros::spinOnce();
	}

	px4_autonomy::Takeoff cmd_tf;
	cmd_tf.take_off = 2;
	cmd_tf.header.stamp = ros::Time::now();
	pub_takeoff.publish(cmd_tf);

	while (status != 1) {
		ros::Duration(1).sleep();
		ROS_INFO("landing...");
		ros::spinOnce();
	}

	ROS_INFO("landed");
}

void takeoff() {
	while (status != 1) {
		ros::Duration(1).sleep();
		ROS_INFO("waitting for OFFBOARD");
		ros::spinOnce();
	}

	px4_autonomy::Takeoff cmd_tf;
	cmd_tf.take_off = 1;
	cmd_tf.header.stamp = ros::Time::now();
	pub_takeoff.publish(cmd_tf);

	while (status != 5) {
		ros::Duration(1).sleep();
		ROS_INFO("taking off...");
		ros::spinOnce();
	}
	int climb_count = 0;
	ROS_INFO("Start climbing");
	while ( HEIGHT - pose_z > 0.1) {
		sendSpd_px4(0,0,0.2,0);
		// sendPose_px4(pose_x, pose_y, pose_z + SPD_CLIMB * climb_count * 0.1, 1.57);
		ros::Duration(0.1).sleep();
		climb_count++;
		ros::spinOnce();
	}
	ROS_INFO("climb finished");

	stop();
	ros::Duration(1).sleep();
}

////////////////////////////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_nav");
	ros::NodeHandle n;

	n.getParam("/uav_nav/spd_max", SPD_MAX);
	n.getParam("/uav_nav/origin_x", MAP_ORIGIN_TO_STRUCT_X);
	n.getParam("/uav_nav/origin_y", MAP_ORIGIN_TO_STRUCT_Y);
	n.getParam("/uav_nav/scan_spd", SCAN_SPEED);
	n.getParam("/uav_nav/scan_delay", SCAN_DELAY);
	n.getParam("/uav_nav/stop_range", STOP_RANGE);
	n.getParam("/uav_nav/safe_range", SAFE_RANGE);
	n.getParam("/uav_nav/brake_coe", BRAKE_COE);
	n.getParam("/uav_nav/waypoint_group", WAYPOINT_GROUP);
	n.getParam("/uav_nav/output_gazebo", OUTPUT_GAZEBO);
	n.getParam("/uav_nav/output_px4", OUTPUT_PX4);
	n.getParam("/uav_nav/height", HEIGHT);
	n.getParam("/uav_nav/spd_climb", SPD_CLIMB);
	n.getParam("/uav_nav/spd_turn", SPD_TURN);
	n.getParam("/uav_nav/angle_tolerance", ANGLE_TOLERANCE);


	if (!wayPointInit()) return 0;

	pub_spd_gazebo = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_cam = n.advertise<mavros_msgs::CameraPose>("/Cam_Pose", 1);
	pub_takeoff = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1);
	pub_spd_px4 = n.advertise<px4_autonomy::Velocity>("/px4/cmd_vel",1);
	pub_pose_px4 = n.advertise<px4_autonomy::Position>("/px4/cmd_pose",1);

	ros::Subscriber sub_laser = n.subscribe("/amcl_pose", 1, laserCB);
	ros::Subscriber sub_status = n.subscribe("/px4/status", 1, statusCB);
	ros::Subscriber sub_pose = n.subscribe("/px4/pose", 1, poseCB);
	ros::Subscriber sub_set_point = n.subscribe("/mavros/setpoint_position/local", 1, setPointCB);
	ros::Rate loop_rate(10);
	ros::Rate oneHz(1);

	while (!gotlaser) {
		oneHz.sleep();
		ROS_INFO("waitting for amcl_pose");
		ros::spinOnce();
	}

	ROS_INFO("get laser, start guide");

	if (OUTPUT_PX4 == true) {
		takeoff();
	}

	ROS_INFO("take off complete");

	while (ros::ok()) {
		// ROS_INFO("%.2f\t%.2f", wayPoints[0].x, wayPoints[0].y);
		ros::spinOnce();

		float disx = wayPoints[progress].x - current_x;
		float disy = wayPoints[progress].y - current_y;
		float dis = sqrt(disx * disx + disy * disy);
		// ROS_INFO("??? %f\t%f", disx, disy);

		if (dis < STOP_RANGE) {
			ROS_INFO("Reach Point #%i \n next: \t%.3f\t%.3f\txplus:%.3f\typlus:%.3f"
				, progress ,wayPoints[progress+1].x, wayPoints[progress+1].y,
				wayPoints[progress+1].x - wayPoints[progress].x,
				wayPoints[progress+1].y - wayPoints[progress].y);
			stop();
			if (wayPoints[progress].need_scan == false) {
				float t = wayPoints[progress].waypoint_delay;
				if (t != 0) {
					ros::Duration(t).sleep();
				}
			} else {
				scan();
			}

			if (wayPoints[progress].set_ori == true) {
				ROS_INFO("turnning...");
				float turn_ori;
				float target_theta = wayPoints[progress].ori;
				float delta_theta = target_theta - current_theta;
				if (delta_theta > 3.1415 || (delta_theta < 0 && delta_theta > -3.1415)) {
					turn_ori = -1;
				} else {
					turn_ori = 1;
				}

				while (	target_theta - current_theta < -ANGLE_TOLERANCE
						||target_theta - current_theta > ANGLE_TOLERANCE) {
					// ROS_INFO("CAL TOLERATE: %.3f", (float)abs(target_theta - current_theta));
					ROS_INFO("%.3f\t%.3f", target_theta, current_theta);
					sendSpd(0,0,turn_ori * SPD_TURN);
					loop_rate.sleep();
					ros::spinOnce();
				}
				stop();
				ros::Duration(1).sleep();
				ROS_INFO("turn finished");
			}
			progress ++;
			continue;
		}

		float brake_coe = BRAKE_COE;
		if (dis < SAFE_RANGE) {
			brake_coe *= dis / SAFE_RANGE;
		}

		float spdx = SPD_MAX / dis * disx * brake_coe;
		float spdy = SPD_MAX / dis * disy * brake_coe;
		sendSpd(spdx, spdy);

		loop_rate.sleep();

		if (progress >= numofWaypoints) {
			stop();
			ROS_INFO("nav finished");
			ros::spinOnce();
			land();
			ros::spin();
		}
	}

	return 0;
}

bool wayPointInit() {
	switch(WAYPOINT_GROUP) {
		case 1:		//standard 4
			addPosPoint(	0, 	1	,true);
			addPosPoint(	1, 	2	,true);
			addPosPoint(	1.7,3.2	);
			setOriForLastPoint(-1.5);
			addPosPoint(	2.7,3.2	);
			addPosPoint(	3, 	2	,true);
			addPosPoint(	4, 	1	,true);
			setOriForLastPoint(3.1);
			addPosPoint( 	7, 	1	);
			addPosPoint( 	7,	2	,true);
			addPosPoint( 	8,	3	,true);
			addPosPoint( 	7.2,1	);
			addPosPoint( 	4.5,1	);
			addPosPoint( 	4, 	3	,true);
			addPosPoint( 	4, 	5	,true);
			addPosPoint( 	7.5,5	);
			addPosPoint( 	7.8,7	,true);
			addPosPoint( 	7.5,5	);
			break;
		case 2:		//map_test
			addPosPoint(1,	1	,true);
			setOriForLastPoint(1.5);
			addPosPoint(4,	1	,true);
			addPosPoint(4.5,-0.38	,true);
			addPosPoint(5.8,-1.05	,true);
			setOriForLastPoint(3.1);
			addPosPoint(7,	-0.82	,true);
			addPosPoint(7.2,0.77	,true);
			addPosPoint(7.2,-1);
			addPosPoint(4.3,-0.6);
			addPosPoint(3.775,0.733);
			addPosPoint(4.84,3.02);
			addPosPoint(5.53,2.9);
			addPosPoint(5.78,5.84,true);
			addPosPoint(6.83,8.75);
			addPosPoint(4.7,8.9);
			addPosPoint(2.4,7);
			addPosPoint(0.375,8.635);
			break;
		case 3:		//map_small_test
			addPosPoint(1.58,1.175	,true);
			addPosPoint(4.77,1.	,true);
			break;

		case 4:		//test_no_stop
			addPosPoint(1,	1	);
			addPosPoint(4,	1	);
			addPosPoint(4.5,-0.38);
			addPosPoint(5.8,-1.05);
			addPosPoint(7,	-0.82);
			addPosPoint(7.2,0.77);
			addPosPoint(7.2,-1);
			addPosPoint(4.3,-0.6);
			addPosPoint(3.775,0.733);
			addPosPoint(4.84,3.02);
			addPosPoint(5.53,2.9);
			addPosPoint(5.78,5.84,true);
			addPosPoint(6.83,8.75);
			addPosPoint(4.7,8.9);
			addPosPoint(2.4,7);
			addPosPoint(0.375,8.635);
			break;

		case 5:		//whole test with stop
			addPosPoint(1,1,true);
			addPosPoint(4,1,true);
			addPosPoint(4.4,0,true);
			setOriForLastPoint(3.1);
			addPosPoint(5.25,-1,true);
			addPosPoint(7.36,-1.2,true);
			addPosPoint(7.34,0.668,true);
			addPosPoint(7.25,-1.2,true);
			addPosPoint(4.56,-0.832,true);
			setOriForLastPoint(1.57);
			addPosPoint(4.277,2,true);
			addPosPoint(5.07,3,true);
			addPosPoint(7.42,2.915,true);
			addPosPoint(7.93,4.41,true);
			addPosPoint(7.48,2.9,true);
			addPosPoint(5.62,2.83,true);
			addPosPoint(5.73,5,true);
			addPosPoint(5.82,5.8,true);
			addPosPoint(6.1,7,true);
			setOriForLastPoint(0);
			addPosPoint(7.4,7.79,true);
			addPosPoint(5.8,8.8,true);
			addPosPoint(3.93,9.1,true);
			addPosPoint(3.43,7.55,true);
			setOriForLastPoint(-1.5);
			addPosPoint(2.27,6.87,true);
			setOriForLastPoint(3.1);
			addPosPoint(1.21,7.75,true);
			addPosPoint(1.332,9.222,true);
			addPosPoint(-0.15,8.8,true);
			break;
			
		case 6:		//whole test without stop
			addPosPoint(1,1);
			addPosPoint(4,1);
			addPosPoint(4.4,0);
			setOriForLastPoint(3.1);
			addPosPoint(5.25,-1);
			addPosPoint(7.36,-1.2);
			addPosPoint(7.34,0.668);
			addPosPoint(7.25,-1.2);
			addPosPoint(4.56,-0.832);
			setOriForLastPoint(1.57);
			addPosPoint(4.277,2);
			addPosPoint(5.07,3);
			addPosPoint(7.42,2.915);
			addPosPoint(7.93,4.41);
			addPosPoint(7.48,2.9);
			addPosPoint(5.62,2.83);
			addPosPoint(5.73,5);
			addPosPoint(5.82,5.8);
			addPosPoint(6.1,7);
			setOriForLastPoint(-1.5);
			addPosPoint(7.4,7.79);
			addPosPoint(5.8,8.8);
			addPosPoint(3.93,9.1);
			addPosPoint(3.43,7.55);
			setOriForLastPoint(0);
			addPosPoint(2.27,6.87);
			setOriForLastPoint(3.1);
			addPosPoint(1.21,7.75);
			addPosPoint(1.332,9.222);
			addPosPoint(-0.15,8.8);
			break;

		default:
			ROS_ERROR("no such waypoint group!!");
			return false;

	}
	return true;
}