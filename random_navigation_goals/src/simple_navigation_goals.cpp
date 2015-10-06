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
using namespace std;

#define PI 3.14159265


void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
     //scan->ranges[] are laser readings
     int x_o = scan->ranges[359];
     
     float coordenadas[scan->ranges.size()][2] = {0};
     
     for (int i = 0; i<scan->ranges.size(); i++){
		 coordenadas[i][0] = sin(scan->angle_min + (scan->angle_increment*i)) * scan->ranges[i];	// x  
		 coordenadas[i][1] = cos(scan->angle_min + (scan->angle_increment*i)) * scan->ranges[i];	// y
	 }
	 
     ROS_INFO("X: [%f]", coordenadas[360][0]);
     ROS_INFO("Y: [%f]", coordenadas[360][1]);
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Subscriber scanSub;
   
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  double min_x_=1.0;
  double max_x_=1.0;
  double min_y_=0;
  double max_y_=0;
  
  tf::TransformListener listener;
  char last_char;
  float hipotenusa=0.0; 
  
while (true){
 	  //we'll send a goal to the robot to move 1 meter forward
	  scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan",10,processLaserScan);
	  goal.target_pose.header.frame_id = "base_link";
	  goal.target_pose.header.stamp = ros::Time::now();

	  srand(time(NULL));
 
	  double x = (double(rand()) / double(RAND_MAX)) * (max_x_-min_x_) + min_x_;
	  //double y = (double(rand()) / double(RAND_MAX)) * (max_y_-min_y_) + min_y_;
	  //double yaw = (double(rand()) / double(RAND_MAX)) * 2*M_PI - M_PI;
	  double 	yaw = 0; double y=0;
	  goal.target_pose.pose.position.x = x;
	  goal.target_pose.pose.position.y = y;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	  ROS_INFO("Sending a new goal to move_base x %lf y %lf yaw %lf", x, y, yaw);
      ac.sendGoal(goal);
	  
	  for (int j=0; j<10; j++){
	        
	      tf::StampedTransform transform;
		  if (listener.frameExists("torso_1")==1 || listener.frameExists("torso_2")==1 || listener.frameExists("torso_3")==1 || listener.frameExists("torso_4")==1 || listener.frameExists("torso_5")==1 || listener.frameExists("torso_6")==1 || listener.frameExists("torso_7")==1){
			   
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
			  
			  hipotenusa = sqrt(x_pessoa*x_pessoa + y_pessoa*y_pessoa);

			  x=0; y=0; yaw=0;	

			  ROS_INFO("Skeleton found!");
			  
			   
			  goal.target_pose.pose.position.x = x;
			  goal.target_pose.pose.position.y = y;
			  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

			  ROS_INFO("Sending a new goal to move_base x %lf y %lf yaw %lf", x, y, yaw);
			  ac.sendGoal(goal);
			  ros::Duration(5.0).sleep();
			   // Classificacao aqui
		  }
			 //catch (tf::TransformException ex){
			   //ROS_ERROR("%s",ex.what());
			   //ros::Duration(1.0).sleep();
			
			 //}

		  //goal.target_pose.pose.position.x = x;
		  //goal.target_pose.pose.position.y = y;
		  //goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

		  //ROS_INFO("Sending a new goal to move_base x %lf y %lf yaw %lf", x, y, yaw);

		  //ac.sendGoal(goal);

		  ac.waitForResult((ros::Duration(1.0)));	// espera s ate verificar se atingiu o goal ou nao
			 
		  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved to goal");
		  //else
			//ROS_INFO("The base failed for some reason");
		
	  }
	  //ros::spinOnce();
}
  return 0;
}
