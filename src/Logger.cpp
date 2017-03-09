#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"		// wird hier als 4-dim Array float64 missbraucht
#include "quadrotor_control/kinematics.h"

#include <fstream>
using namespace std;
ofstream logFile;

ros::Time start;

double VSoll[]  = {0.0, 0.0, 0.0, 0.0};
double VIst[]   = {0.0, 0.0, 0.0, 0.0};
double PoseIst[] = {0.0, 0.0, 0.0};
double VModel[] = {0.0, 0.0, 0.0, 0.0};
double PoseModel[] = {0.0, 0.0, 0.0};

void callback_VSoll( const geometry_msgs::Quaternion::ConstPtr& msg ){
	VSoll[0] = msg->x;
	VSoll[1] = msg->y;
	VSoll[2] = msg->z;
	VSoll[3] = msg->w;
}

void callback_VIst( const quadrotor_control::kinematics::ConstPtr& msg ){
	VIst[0] = msg->vel.linear.x;
	VIst[1] = msg->vel.linear.y;
	VIst[2] = msg->vel.linear.z;
	VIst[3] = msg->vel.angular.z;
	PoseIst[0] = msg->pose.orientation.x;
	PoseIst[1] = msg->pose.orientation.y;
	PoseIst[2] = msg->pose.orientation.z;
}

void callback_VModel( const quadrotor_control::kinematics::ConstPtr& msg ){
	VModel[0] = msg->vel.linear.x;
	VModel[1] = msg->vel.linear.y;
	VModel[2] = msg->vel.linear.z;
	VModel[3] = msg->vel.angular.z;
	PoseModel[0] = msg->pose.orientation.x;
	PoseModel[1] = msg->pose.orientation.y;
	PoseModel[2] = msg->pose.orientation.z;
}



int main(int argc, char **argv)
{
 	
	ros::init(argc, argv, "Logger");

	ros::NodeHandle nh("Logger");

	

	

	ros::Subscriber subVSoll  = nh.subscribe("/VSoll", 10, callback_VSoll);

	ros::Subscriber subVIst		= nh.subscribe("/kin_measure", 10, callback_VIst);

	ros::Subscriber subVModel	= nh.subscribe("/kin_model", 10, callback_VModel);

	char filePathName[] = "/home/youbot/Desktop/logV.txt";
	logFile.open(filePathName); 
	if(!logFile.is_open()){
			ROS_ERROR("Logfile: '%s' konnte nicht geöffnet werden. Beende.", filePathName);
			return 0;
	}
	logFile << " Zeit , VSollX , VSollY, VSollZ, VSollYaw, VIstX  , VIstY  , VIstZ  , VIstYaw, PhiIST, ThetaIst, PsiIst ,VModelX, VModelY , VModelZ , VModelYaw, PhiModel, ThetaModel, PsiModel" << std::endl; 


	ROS_INFO("Starte Logging");

	ros::Rate loop_rate(100);
	start = ros::Time::now();
	while(ros::ok()){
		logFile << (ros::Time::now() - start ).toSec() << " , ";
		for( int i = 0; i < 4; i++ )
			logFile << VSoll[i] << " , ";

		for( int i = 0; i < 4; i++ )
			logFile << VIst[i] << " , ";		
		for( int i = 0; i < 3; i++ )
			logFile << PoseIst[i] << " , ";

		for( int i = 0; i < 4; i++ )
			logFile << VModel[i] << " , ";
		for( int i = 0; i < 3; i++ ){
			logFile << PoseModel[i];
			if( i != 2 ) 	logFile << " , ";
			else					logFile << std::endl;
		}
		

  	ros::spinOnce();
  	loop_rate.sleep();
	}

	return 0;
}
