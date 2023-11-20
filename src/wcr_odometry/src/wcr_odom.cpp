#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <array>
#include <cmath>

float v_x = 0.0;
float v_y = 0.0;
float omega = 0.0;

float a = 0.225/2;
float b = 0.225/2;
float r = 0.0254;
float x_w_r[4] = {a, -a, -a, a};
float y_w_r[4] = {b, b, -b, -b};
float W[4];
float q[8];



void jointStateCallback(const sensor_msgs::JointState& state)
{	
	q[0] = state.velocity[5];
	q[1] = state.velocity[1];
	q[2] = state.velocity[3];
	q[3] = state.velocity[7];
	q[4] = state.position[4];
	q[5] = state.position[0];
	q[6] = state.position[2];
	q[7] = state.position[6];
	
	v_x = 0.0;
	v_y = 0.0;
	omega = 0.0;
	
	for(int i =0;i<4;i++){
		v_x = v_x + (cos(q[i+4])/4)*(q[i]*r);
		v_y = v_y + (sin(q[i+4])/4)*(q[i]*r);
		W[i] = (-y_w_r[i]*cos(q[i+4]) + x_w_r[i]*sin(q[i+4]))/(4*pow(x_w_r[i], 2) + 4*pow(y_w_r[i], 2));
		omega = omega + W[i]*q[i]*r;
	}	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wcr_odom");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("wcr/joint_states", 1000, jointStateCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  
  float x = 0.0;
	float y = 0.0;
	float th = 0.0;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate loop_rate(100);
  	ros::spinOnce();
	
	while(n.ok()){
		current_time = ros::Time::now();
		float dt = (current_time - last_time).toSec();
		float delta_x = (v_x * cos(th) - v_y * sin(th)) * dt;
    		float delta_y = (v_x * sin(th) + v_y * cos(th)) * dt;
    float delta_th = omega * dt;
    
    x += delta_x;
    y += delta_y;
    th += delta_th;
    
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    
    //slanje na tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);
    
    //slanje na odom
    
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.linear.y = v_y;
    odom.twist.twist.angular.z = omega;
    
    odom_pub.publish(odom);

    last_time = current_time;
		
		ros::spinOnce();
    loop_rate.sleep();
    
	}
  

  return 0;
}

