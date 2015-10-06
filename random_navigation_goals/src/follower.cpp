#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <cstdio>
#include <iostream>
#include <cstring>
#include <string>
#include <math.h> 
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sound_play/sound_play.h>
#include <unistd.h>

using namespace std;

#define PI 3.14159265
double xe= 2.5, ye=0;
double p11=1, p12=0, p21=0, p22=1;
double k11=0, k12=0, k21=0, k22=0;
double r11=0.1, r12=0, r21=0, r22=0.1;
double t=1;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  if(msg->data.find("follow")!=std::string::npos){
	  system("rosrun sound_play say.py 'ok'");
	  system("rosnode kill /simple_navigation_goals");
	  ros::NodeHandle n;
	  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	  //tell the action client that we want to spin a thread by default
	  MoveBaseClient ac("move_base", true);

      //wait for the action server to come up
      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;
  
	  tf::TransformListener listener;
	  char last_char;
	  double hipotenusa=0.0; 
	  double x; double y; double yaw;
	  double x_pessoa_anterior=0; double y_pessoa_anterior=0; double yaw_pessoa_anterior=0; 
	  int conta = 0;
	  double vx, vy, vel_calc, vel_anterior=0;
	  double w;
	  geometry_msgs::Twist msg;
	  double incr;
	  double x_pessoa, y_pessoa, z_pessoa;
	  double roll_pessoa, pitch_pessoa, yaw_pessoa;
  
	  while (true){
          
          goal.target_pose.header.frame_id = "base_link";
	      goal.target_pose.header.stamp = ros::Time::now();

	      tf::StampedTransform transform;
		  if (listener.frameExists("torso_1")==1 || listener.frameExists("torso_2")==1 || listener.frameExists("torso_3")==1 || listener.frameExists("torso_4")==1 || listener.frameExists("torso_5")==1){
			   
		      vector <string> ids;
              listener.getFrameStrings(ids); 
			   
			  for(vector<string>::iterator i = ids.begin(); i < ids.end(); i++){
                  string elem = *i;
                  if (strstr(elem.c_str(),"torso_") != NULL){
                      last_char = elem[elem.length()-1];
                  }  
              }
              
                            
       		  listener.lookupTransform("openni_depth_frame", "torso_" + boost::lexical_cast<std::string>(int(last_char)-'0'), ros::Time(0), transform);
              double x_pessoa = transform.getOrigin().x();
 		      double y_pessoa = transform.getOrigin().y();
		      double z_pessoa = transform.getOrigin().z(); 
		      
		      double roll_pessoa, pitch_pessoa, yaw_pessoa;
		      transform.getBasis().getRPY(roll_pessoa, pitch_pessoa, yaw_pessoa);
			  ROS_INFO("Coordenadas da pessoa: x %lf y %lf yaw %lf", x_pessoa, y_pessoa, yaw_pessoa); 
			  
			  if (conta == 0){
				  vx=0; vy=0;
			  }
		      else{
		          vx = (x_pessoa-x_pessoa_anterior)/0.5;
		          vy = (y_pessoa-y_pessoa_anterior)/0.5;
			  }
			  
			  hipotenusa = sqrt(x_pessoa*x_pessoa + y_pessoa*y_pessoa);

			  x=x_pessoa - 2.5; y=0; yaw=asin(y_pessoa/hipotenusa);	
			  		 
			  if (x<0.2)
			      x=0;
			      
			  if (yaw<0.15 && yaw>-0.15)
				  yaw=0;
			      
			  ROS_INFO("Skeleton found!");
              if (x_pessoa==x_pessoa_anterior){
			      x=0;
			      y=0;
			      yaw=0;	
			  }
			   
			  goal.target_pose.pose.position.x = x;
			  goal.target_pose.pose.position.y = y;
			  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
			  
			  xe = xe + vx*t;
			  ye = ye + vy*t;
			  xe = xe + k11*(x_pessoa-xe) + k12*(y_pessoa-ye);
			  ye = ye + k21*(x_pessoa-xe) + k22*(y_pessoa-ye);
			  k11 = (p11*p22 - p12*p21 + p11*r22 - p12*r21)/(p11*p22 - p12*p21 + p11*r22 - p12*r21 - p21*r12 + p22*r11 + r11*r22 - r12*r21);
			  k12 = -(p11*r12 - p12*r11)/(p11*p22 - p12*p21 + p11*r22 - p12*r21 - p21*r12 + p22*r11 + r11*r22 - r12*r21);
			  k21 = (p21*r22 - p22*r21)/(p11*p22 - p12*p21 + p11*r22 - p12*r21 - p21*r12 + p22*r11 + r11*r22 - r12*r21);
			  k22 = (p11*p22 - p12*p21 - p21*r12 + p22*r11)/(p11*p22 - p12*p21 + p11*r22 - p12*r21 - p21*r12 + p22*r11 + r11*r22 - r12*r21);
			  p11 = (p11*p22*r11 - p12*p21*r11 + p11*r11*r22 - p11*r12*r21)/(p11*p22 - p12*p21 + p11*r22 - p12*r21 - p21*r12 + p22*r11 + r11*r22 - r12*r21);
			  p12 = (p11*p22*r12 - p12*p21*r12 + p12*r11*r22 - p12*r12*r21)/(p11*p22 - p12*p21 + p11*r22 - p12*r21 - p21*r12 + p22*r11 + r11*r22 - r12*r21);
			  p21 = (p11*p22*r21 - p12*p21*r21 + p21*r11*r22 - p21*r12*r21)/(p11*p22 - p12*p21 + p11*r22 - p12*r21 - p21*r12 + p22*r11 + r11*r22 - r12*r21);
			  p22 = (p11*p22*r22 - p12*p21*r22 + p22*r11*r22 - p22*r12*r21)/(p11*p22 - p12*p21 + p11*r22 - p12*r21 - p21*r12 + p22*r11 + r11*r22 - r12*r21); 
			  
			  
			  ROS_INFO("Coordenadas estimadas: x %lf y %lf", xe, ye); 
			  if (xe<0.75 ){
					  for(int k=0;k<100;k++){
						msg.linear.x = -0.3;	 
						msg.angular.z = -0.5;	
						chatter_pub.publish(msg);	
						ros::Duration(0.04).sleep(); 
					  }
					  for(int l=0;l<100;l++){
					      msg.linear.x = 0;	 
					      msg.angular.z = 0;	
					      chatter_pub.publish(msg);	
					      ros::Duration(0.03).sleep(); 
					  }
		            
				  }
			  ros::spinOnce();
				

			  ROS_INFO("Sending a new goal to move_base x %lf y %lf yaw %lf", x, y, yaw);
			  ac.sendGoal(goal);
			  ac.waitForResult((ros::Duration(0.1)));	// espera s ate verificar se atingiu o goal ou nao
			 
		      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			      ROS_INFO("Hooray, the base moved to goal");
			   
			  x_pessoa_anterior=x_pessoa;
			  y_pessoa_anterior=y_pessoa;
			  yaw_pessoa_anterior = yaw;
			  vel_anterior = vx;
			  conta ++;
		  }
	}		  			
}  
}



int main(int argc, char** argv){
  ros::init(argc, argv, "follower");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("recognizer/output", 1000, Callback);
  ros::spin();
  return 0;
}
