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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "follower");
  ros::NodeHandle n;
  ros::Subscriber scanSub;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   
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
  double vx, vel_calc, vel_anterior=0;
  double w;
  geometry_msgs::Twist msg;
  double incr;
  double roll_pessoa, pitch_pessoa, yaw_pessoa;
  double x_pessoa, y_pessoa, z_pessoa;
  
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
              
			 for (int i=0; i<50; i++){
				   
				  listener.lookupTransform("openni_depth_frame", "torso_" + boost::lexical_cast<std::string>(int(last_char)-'0'), ros::Time(0), transform);
              x_pessoa = transform.getOrigin().x();
 		      y_pessoa = transform.getOrigin().y();
		      z_pessoa = transform.getOrigin().z(); 
		      
		      
		      transform.getBasis().getRPY(roll_pessoa, pitch_pessoa, yaw_pessoa);
			  ROS_INFO("Coordenadas da pessoa: x %lf y %lf yaw %lf", x_pessoa, y_pessoa, yaw_pessoa); 
			  
			  hipotenusa = sqrt(x_pessoa*x_pessoa + y_pessoa*y_pessoa);

			  x=x_pessoa-2.0; y=0; yaw=asin(y_pessoa/hipotenusa);	
			  
			      
			  if (conta == 0)
				  vx=0;
		      else
		          vx = (x_pessoa-x_pessoa_anterior)/1;
				  //vx = sqrt((x_pessoa-x_pessoa_anterior)*(x_pessoa-x_pessoa_anterior) + (y_pessoa-y_pessoa_anterior)*(y_pessoa-y_pessoa_anterior))/0.5; w = (yaw-yaw_pessoa_anterior)/0.5;
			  vel_calc = vx;
			  if (vx<0.1)
			      vx=0;
			  //else if (vx>0.2)
			    //  vx = 0.2;    
			  		 
			  if (x<0.2 && x>-0.2)
			      vx = 0;
			      
			  else if(x<-0.2 && vx==0)
			 	  vx=-0.2;
			  else if (x<-0.2)
			      vx = -vx;
			  //else if((x_pessoa-x_pessoa_anterior)<-0.2 && x>0.2)
			 	//  vx = 0;
			  else if(x>0.2 && vx==0)
			      vx=0.2;
			  else
			 	  vx = vx; //vx=0.2;
			 	  
			  if (yaw<0.2 && yaw>-0.2)
			      w= 0;
			      
			  else if (yaw<-0.2)
			 	  w = -0.2;
			  else
			 	  w = 0.2;	  
			  
			  
				  if(vx==vel_anterior){
					  msg.linear.x = vx;	 
					  msg.angular.z = w;	
					  chatter_pub.publish(msg);
				  }
				  else if(vx<vel_anterior){
					  incr = (vx-vel_anterior)/50.0;
					  ROS_INFO("INC: %lf", incr);
					  msg.linear.x = vel_anterior+(i*incr);	 
					  msg.angular.z = w;	
					  chatter_pub.publish(msg);				  
				  }
				  else{
					  incr = (vx-vel_anterior)/50.0;
					  ROS_INFO("INC: %lf", incr);
					  msg.linear.x = vel_anterior+(incr*i);	 
					  msg.angular.z = w;	
					  chatter_pub.publish(msg);		  
				  }
				  ROS_INFO("Coordenadas da pessoa: x %lf y %lf yaw %lf", x_pessoa, y_pessoa, yaw_pessoa); 
				  ros::Duration(0.02).sleep(); 
			  }
			  
              ros::spinOnce();
			  ROS_INFO("Skeleton found!");
			  ROS_INFO("Velocidade: %lf		X anterior: %lf", vx, x_pessoa_anterior);
			  
			  x_pessoa_anterior=x_pessoa;
			  y_pessoa_anterior=y_pessoa;
			  yaw_pessoa_anterior = yaw;
			  vel_anterior = vx;
			  conta ++;
			  //ros::Duration(1).sleep();
		  }
		  			
}
  return 0;
}
