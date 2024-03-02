#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
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
float W_c[4];
float q[8];

//POJAČANJA REGULATORA
float k_x = 0.5;
float k_y = 0.5;
float k_th = 0.1;

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
		v_x =v_x + (cos(q[i+4])/4)*(q[i]*r);
		v_y =v_y + (sin(q[i+4])/4)*(q[i]*r);
		W[i] = (-y_w_r[i]*cos(q[i+4]) + x_w_r[i]*sin(q[i+4]))/(4*pow(x_w_r[i], 2) + 4*pow(y_w_r[i], 2));
		omega = omega + W[i]*q[i]*r;
	}	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nonlinear_trajectory_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/wcr/joint_states", 1000, jointStateCallback);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/wcr/cmd_vel", 10);
  
  geometry_msgs::Twist cmd_vel_msg;
  geometry_msgs::PointStamped trajectory_msg;
  
  float x = 0.0;
	float y = 0.0;
	float th = 0.0;
	
	float normalized_time = 0.0;
	
	float last_x_d = 0.0;
	float last_y_d = 0.0;
	float last_th_d = 0.0;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate loop_rate(100);
  	ros::spinOnce();
	
	while(n.ok()){		
		current_time = ros::Time::now();

		//promjena pozicije		
		float dt = (current_time - last_time).toSec();
		float delta_x = (v_x * cos(th) - v_y * sin(th)) * dt;
    	float delta_y = (v_x * sin(th) + v_y * cos(th)) * dt;
    	float delta_th = omega * dt;
    
    	normalized_time += dt;
    
		//trenutna pozicija
		x += delta_x;
		y += delta_y;
		th += delta_th;
		
		// ulazna trajektorija
		float x_d = 0.5*sin(0.05*normalized_time);
		float y_d = 0.5*(1 - cos(0.05*normalized_time));
		//float y_d = 0;
		//float th_d = atan2(y_d, x_d);
		float th_d = 0.0;
		
		// derivacija ulazne trajektorije
		float v_x_d = (x_d - last_x_d)/dt;
		float v_y_d = (y_d - last_y_d)/dt;
		float v_th_d = (th_d - last_th_d)/dt;
		
		// rotacija derivirane trajektorije
		float v_x_d_rot = v_x_d*cos(th_d) + v_y_d*sin(th_d);
		float v_y_d_rot = -v_x_d*sin(th_d) + v_y_d*cos(th_d);
		v_x_d = v_x_d_rot;
		v_y_d = v_y_d_rot;
		
		// greška pozicije
		float e_x = x_d - x;
		float e_y = y_d - y;
		float e_th = th_d - th;
		float e_x_rot = e_x*cos(th) + e_y*sin(th);
		float e_y_rot = -e_x*sin(th) + e_y*cos(th);
		e_x = e_x_rot;
		e_y = e_y_rot;
		
		float v_x_i_d[4];
		float v_y_i_d[4];
		float v_th_i_d[4];
		float v_d[4];
		float delta_d[4];
		float a[4];
		float b[4];
		
		float v_c[4];
		float delta_c[4];
		
		float v_x_c = 0.0;
		float v_y_c = 0.0;
		float v_th_c = 0.0;
		
		for(int i = 0; i < 4;i++){
			v_x_i_d[i] = v_x_d - y_w_r[i]*v_th_d;
			v_y_i_d[i] = v_y_d + x_w_r[i]*v_th_d;
			v_d[i] = sqrt(pow(v_x_i_d[i], 2) + pow(v_y_i_d[i], 2));
			delta_d[i] = atan2(v_y_i_d[i], v_x_i_d[i]);
			a[i] = v_d[i]*cos(delta_d[i]) + k_x*e_x - k_th*y_w_r[i]*e_th;
			b[i] = v_d[i]*sin(delta_d[i]) + k_y*e_y + k_th*x_w_r[i]*e_th;
			
			float z_1;
			float z_2;
			
			if(abs(delta_d[i] - q[4+i]) <= M_PI/2){
				z_1 = 1;
				z_2 = 0;
			}
			else{
				z_1 = -1;
				z_2 = M_PI;
			}
			
			v_c[i] = z_1*sqrt(pow(a[i], 2) + pow(b[i], 2));
			delta_c[i] = z_2 + atan2(b[i], a[i]);
			
			v_x_c += (cos(delta_c[i])/4)*(v_c[i]);
			v_y_c += (sin(delta_c[i])/4)*(v_c[i]);
			W_c[i] = (-y_w_r[i]*cos(delta_c[i]) + x_w_r[i]*sin(delta_c[i]))/(4*pow(x_w_r[i], 2) + 4*pow(y_w_r[i], 2));
			v_th_c +=  W_c[i]*v_c[i];
		}
		
	/*if(v_x_c > 0.1)	
		cmd_vel_msg.linear.x = 0.1;
	else if (v_x_c < -0.1)
		cmd_vel_msg.linear.x = -0.1;
	else
		cmd_vel_msg.linear.x = v_x_c;
		
	if(v_y_c > 0.1)	
		cmd_vel_msg.linear.y = 0.1;
	else if (v_x_c < -0.1)
		cmd_vel_msg.linear.y = -0.1;
	else
		cmd_vel_msg.linear.y = v_y_c;
		
	if(v_th_c > 0.1)	
		cmd_vel_msg.angular.z = 0.1;
	else if (v_x_c < -0.1)
		cmd_vel_msg.angular.z = -0.1;
	else
		cmd_vel_msg.angular.z = v_th_c;
	*/
	cmd_vel_msg.linear.x = v_x_c;
	cmd_vel_msg.linear.y = v_y_c;
	cmd_vel_msg.angular.z = v_th_c;
	cmd_vel_pub.publish(cmd_vel_msg);

    last_time = current_time;
    last_x_d = x_d;
    last_y_d = y_d;
    last_th_d = th_d;
		
	ros::spinOnce();
    loop_rate.sleep();
    
	}
  

  return 0;
}

