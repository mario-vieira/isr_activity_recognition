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
using namespace std;

#define PI 3.14159265

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double xe= 2.5, ye=0;
double p11=1, p12=0, p21=0, p22=1;
double k11=0, k12=0, k21=0, k22=0;
double r11=0.1, r12=0, r21=0, r22=0.1;
double t=0.04;

void Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  if(msg->data.find("follow")!=std::string::npos){
	  system("rosrun sound_play say.py 'ok'");
	  //system("rosnode kill /simple_navigation_goals");
	  ros::NodeHandle n;
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
              x_pessoa = transform.getOrigin().x();
 		      y_pessoa = transform.getOrigin().y();
		      z_pessoa = transform.getOrigin().z(); 
		      
		      
		      transform.getBasis().getRPY(roll_pessoa, pitch_pessoa, yaw_pessoa);
		      
		      if (conta == 0){
				  vx=0; vy=0;
			  }
		      else{
		          vx = (x_pessoa-x_pessoa_anterior)/1;
		          vy = (y_pessoa-y_pessoa_anterior)/1;
			  }
			  vel_calc = vx;
			  		  
			  hipotenusa = sqrt(x_pessoa*x_pessoa + y_pessoa*y_pessoa);

			  x=x_pessoa-2.0; y=0; yaw=asin(y_pessoa/hipotenusa);	
			  
			  //vx = sqrt((x_pessoa-x_pessoa_anterior)*(x_pessoa-x_pessoa_anterior) + (y_pessoa-y_pessoa_anterior)*(y_pessoa-y_pessoa_anterior))/0.5; w = (yaw-yaw_pessoa_anterior)/0.5;
			  
			  if (vx<0.1)
			      vx=0;   
			  		 
			  if (x<0.2 && x>-0.2)
			      vx = 0;
			      
			  else if(x<-0.2 && vx==0)
			 	  vx=-0.2;
			  else if (x<-0.2)
			      vx = -vx;
			  else if (x_pessoa==x_pessoa_anterior){
			      vx=0;
			      w=0;
			  }
			  else if(x>0.2 && vx==0)
			      vx=0.2;
			  
			  else
			 	  /*vx = vx;*/ vx=0.2;
			 	  
			  if (yaw<0.2 && yaw>-0.2)
			      w= 0;
			      
			  else if (yaw<-0.2)
			 	  w = -0.2;
			  else
			 	  w = 0.2;	  
			  
			  
			  
			  for (int i=0; i<50; i++){
			  
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
				  xe = xe + vel_calc*t;
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
				  
				  if (xe<1 ){
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
		              break;
				  }

		      
				  ROS_INFO("Coordenadas da pessoa: x %lf y %lf yaw %lf", x_pessoa, y_pessoa, yaw_pessoa); 
				  ROS_INFO("Coordenadas estimadas: x %lf y %lf", xe, ye); 
				  ros::Duration(0.02).sleep(); 
			  }
			  
              ros::spinOnce();
			  ROS_INFO("Skeleton found!");
			  ROS_INFO("Velocidade: %lf		X anterior: %lf", vel_calc, x_pessoa_anterior);
			  
			  x_pessoa_anterior=x_pessoa;
			  y_pessoa_anterior=y_pessoa;
			  yaw_pessoa_anterior = yaw;
			  vel_anterior = vx;
			  conta ++;
			  //ros::Duration(1).sleep();
		  }
	}		  			
}
}


int main(int argc, char** argv){
  ros::init(argc, argv, "follower_kalman");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("recognizer/output", 1000, Callback);
  ros::spin();   

  return 0;
}
