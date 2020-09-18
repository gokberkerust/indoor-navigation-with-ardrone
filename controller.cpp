#include "ros/ros.h"
#include <signal.h>
#include "std_msgs/String.h"
#include <ardrone_autonomy/Navdata.h>
#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
using namespace std;

geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_altitude;
std_msgs::Empty emp_msg;
ardrone_autonomy::Navdata msg_in;

double maxVelX = 0.03;
double maxVelY = 0.01;
double maxVelZ = 0.3;
double maxAngZ = 0.1;
string calib = "";
string vision = "";
string local = "";
float altd;
int status;
int lastMove;
int g_request_shutdown = 0;

void mySigIntHandler(int sig){
  	g_request_shutdown = 1;
}


void calibCallback(const std_msgs::String::ConstPtr& msg){
	calib = msg -> data.c_str();
}

void visionCallback(const std_msgs::String::ConstPtr& msg){
    vision = msg -> data.c_str();
}

void localCallback(const std_msgs::String::ConstPtr& msg){
	local = msg -> data.c_str();
}


void navdataCallback(const ardrone_autonomy::Navdata& msg_in){
	status = msg_in.state;
	altd = msg_in.altd;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "controller",ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);
	ros::NodeHandle n;

	ros::Publisher pubTakeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	ros::Publisher pubLand = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
	ros::Publisher pubVelocity = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::Subscriber sub1 = n.subscribe("/ardrone/calibration_response", 1, calibCallback);
	ros::Subscriber sub2 = n.subscribe("/ardrone/humandetection",1, visionCallback);
	ros::Subscriber sub3 = n.subscribe("/ardrone/localization",1, localCallback);
	ros::Subscriber sub4 = n.subscribe("/ardrone/navdata", 1, navdataCallback);


	//hover message
	twist_msg_hover.linear.x=0.0; 
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0; 
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0; 
	//altitude message
	twist_msg_altitude.linear.x=0.0; 
	twist_msg_altitude.linear.y=0.0;
	twist_msg_altitude.linear.z=maxVelZ;
	twist_msg_altitude.angular.x=0.0; 
	twist_msg_altitude.angular.y=0.0;
	twist_msg_altitude.angular.z=0.0; 
	//vel message
	twist_msg.linear.x=0.0; 
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0; 
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0; 



	ros::Duration(1.0).sleep();
	ros::spinOnce();
	cout << "status: " << status << "\n";
	if(status == 2){
		pubTakeoff.publish(emp_msg);
	}
	ros::Duration(8.0).sleep();
	cout << "waited." << "\n";


	while(calib != "calibrated" && local.length() == 0){
			cout << "calibration1..." << "\n";

			if(g_request_shutdown){
				pubVelocity.publish(twist_msg_hover);
				pubLand.publish(emp_msg);
				ros::shutdown();
				return 0;
			}
			
			if(calib == "right"){
				cout << "going right" << "\n" ;
			//	if(lastMove != 0){
				twist_msg.linear.y = -(maxVelY*2);
				pubVelocity.publish(twist_msg); 
			//		lastMove = 0;
			//	}				
			}else if(calib == "left"){
				cout << "going left" << "\n" ;
				//if(lastMove != 1 ){
				twist_msg.linear.y = maxVelY;
				pubVelocity.publish(twist_msg); 

			//		lastMove = 1 ;
			//	}
			}
			ros::Duration(0.5).sleep();
			pubVelocity.publish(twist_msg_hover);
			ros::spinOnce();
	}
	pubVelocity.publish(twist_msg_altitude); 
	ros::Rate r(10);
	while (altd < 1200){
		if(g_request_shutdown){
				pubVelocity.publish(twist_msg_hover);
				pubLand.publish(emp_msg);
				ros::shutdown();
				return 0;
		}
		cout<< "getting altitude: " << altd << "\n";
		ros::spinOnce();
		r.sleep();
	}
	pubVelocity.publish(twist_msg_hover);
	ros::Duration(0.5).sleep();



	/*
		the controller of drone
	*/

	//ros::Rate r(10);
	while (!g_request_shutdown){
		

		if(g_request_shutdown){
			pubVelocity.publish(twist_msg_hover);
			pubLand.publish(emp_msg);
			ros::shutdown();
			return 0;
		}
		

		int lastMove = -1;
		cout << "calibration message: " << calib << "\n";
		while(calib != "calibrated" && local.length() == 0){
			cout << "calibration..." << "\n";

			if(g_request_shutdown){
				pubVelocity.publish(twist_msg_hover);
				pubLand.publish(emp_msg);
				ros::shutdown();
				return 0;
			}
			
			if(calib == "right"){
				cout << "going right" << "\n" ;
			//	if(lastMove != 0){
				twist_msg.linear.y = -(maxVelY*2);
				pubVelocity.publish(twist_msg); 
			//		lastMove = 0;
			//	}				
			}else if(calib == "left"){
				cout << "going left" << "\n" ;
				//if(lastMove != 1 ){
				twist_msg.linear.y = maxVelY;
				pubVelocity.publish(twist_msg); 

			//		lastMove = 1 ;
			//	}
			}
			ros::Duration(0.5).sleep();
			pubVelocity.publish(twist_msg_hover);
			ros::spinOnce();
		}

		pubVelocity.publish(twist_msg_hover);
		ros::Duration(0.5).sleep();

		cout << "calibration message: " << calib << "\n";
		cout << "vision message: " << vision << "\n";
		// set velocity to move forward
		twist_msg.linear.x = maxVelX;
		twist_msg.linear.y = 0.00;
		pubVelocity.publish(twist_msg); 
		ros::Duration(0.5).sleep();
		int prevMove = 0; // check for the last move,
		while(calib == "calibrated" && local.length() == 0){

			if(g_request_shutdown){
				pubVelocity.publish(twist_msg_hover);
				pubLand.publish(emp_msg);
				ros::shutdown();
				return 0;
			}



			if(vision == "Detected"){
				pubVelocity.publish(twist_msg_hover);
				ros::Duration(0.5).sleep();
				cout << "Human detected waiting for him" << "\n";
				prevMove = 0;
			}else if(vision == "Clear"){
				cout << "moving forward..." << "\n";
				if(prevMove == 0){
					cout << "getting speed up..." << "\n";
					twist_msg.linear.x = maxVelX;
					twist_msg.linear.y = 0.00;
					pubVelocity.publish(twist_msg); 
					ros::Duration(0.5).sleep();
					prevMove = 1;
				}
				else{
					ros::Duration(0.5).sleep();
				}
			}
			ros::spinOnce();
		}

		while(local.length() > 0 && local != "done"){
			twist_msg.linear.x = 0;
			
			if(g_request_shutdown){
				pubVelocity.publish(twist_msg_hover);
				pubLand.publish(emp_msg);
				ros::shutdown();
				return 0;
			}
			twist_msg.linear.x = 0;
			pubVelocity.publish(twist_msg_hover);
			ros::Duration(0.5).sleep();

			if(local == "right"){
				cout << "arranging right" << "\n" ;
			//	if(lastMove != 0){
				twist_msg.linear.y = -maxVelY;
				pubVelocity.publish(twist_msg); 
			//		lastMove = 0;
			//	}				
			}else if(local == "left"){
				cout << "aranging left" << "\n" ;
				//if(lastMove != 1 ){
				twist_msg.linear.y = maxVelY;
				pubVelocity.publish(twist_msg); 

			//		lastMove = 1 ;
			//	}
			}
			ros::Duration(0.5).sleep();
			pubVelocity.publish(twist_msg_hover);
			ros::spinOnce();
		}
		int counter = 0 ;
		while(local.length() > 0 && local == "done"){
			cout << "arranged!" << "\n";		
			if(g_request_shutdown){
				pubVelocity.publish(twist_msg_hover);
				pubLand.publish(emp_msg);
				ros::shutdown();
				return 0;
			}

			pubVelocity.publish(twist_msg_hover);
			twist_msg.linear.y = 0;
			twist_msg.linear.x = 0;
			twist_msg.angular.z = maxAngZ;
			counter += 1;
			if(counter == 5){
				cout << "turning right" << "\n";
				pubVelocity.publish(twist_msg);
				ros::Duration(3.0).sleep();
				pubVelocity.publish(twist_msg_hover);
				ros::Duration(0.5).sleep();
				cout << "landing... " << "\n";
				pubLand.publish(emp_msg);
				ros::shutdown();
				return 0;
			}


		}






	  	ros::spinOnce();            
	  	r.sleep();
	}
	if(g_request_shutdown){
				pubVelocity.publish(twist_msg_hover);
				pubLand.publish(emp_msg);
				ros::shutdown();
				return 0;
	}

	return 0 ;

}