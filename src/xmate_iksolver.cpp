#include <kdl/kdl.hpp> 
#include <kdl/chain.hpp> 
#include <kdl/tree.hpp> 
#include <kdl/segment.hpp> 
#include <kdl/chainfksolver.hpp> 
#include <kdl_parser/kdl_parser.hpp> 
#include <kdl/chainfksolverpos_recursive.hpp> 
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp> 
#include <kdl/utilities/error.h>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>

#include <stdio.h> 
#include <iostream> 
#include <sys/times.h>
#include <unistd.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#define NO_OF_JOINTS 13

using namespace KDL; 
using namespace std; 

bool new_target = false;
geometry_msgs::Twist target;
sensor_msgs::JointState joint_state;


const char* j_name_list[]={
"xmate_joint_1", 
"xmate_joint_2", 
"xmate_joint_3", 
"xmate_joint_4", 
"xmate_joint_5", 
"xmate_joint_6",
"xmate_joint_7", 
"finger_joint", 
"left_inner_knuckle_joint", 
"left_inner_finger_joint",
"right_outer_knuckle_joint", 
"right_inner_knuckle_joint", 
"right_inner_finger_joint"
};


void Joint_State_Msg_Initialize(int size, char* joint_name_list[]){
    int i;
    joint_state.name.resize(size);
    joint_state.position.resize(size);
    for(i = 0; i < size; i++)
        joint_state.name[i] = joint_name_list[i];

}

void posmsgCallback(const geometry_msgs::Twist::ConstPtr&  msg)
{
	new_target = true;
	target.linear.x = msg->linear.x;
	target.linear.y = msg->linear.y;
	target.linear.z = msg->linear.z;

	target.angular.x = msg->angular.x;
	target.angular.y = msg->angular.y;
	target.angular.z = msg->angular.z;

	ROS_INFO("x:[%f] y:[%f] z:[%f]", target.linear.x, target.linear.y, target.linear.z);
	ROS_INFO("tx:[%f] ty:[%f] tz:[%f]", target.angular.x, target.angular.y, target.angular.z);
}

int main(int argc,char** argv){
	ros::init(argc, argv, "ik_solver");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	//ros::Publisher jointstates_publisher = n.advertise<sensor_msgs::JointState>("joint_cmd", 1000);
	ros::Publisher jointstates_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
   
	ros::Subscriber tarpos_sub = n.subscribe("tarpos_pub", 1000, posmsgCallback);
	
	Joint_State_Msg_Initialize(NO_OF_JOINTS,(char**)j_name_list);

	Tree my_tree; 
	kdl_parser::treeFromFile("/home/robot/catkin_ws/src/exoskeleton_package/xmate3_description/urdf/xmate3_with_gripper.urdf",my_tree);
	bool exit_value; 
	Chain chain; 
	exit_value = my_tree.getChain("world","tcp_link",chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	//ChainJntToJacSolver jac_solver = ChainJntToJacSolver(chain);
	ChainJntToJacSolver jac_solver(chain);
	unsigned int nj = chain.getNrOfJoints(); 
	unsigned int ns = chain.getNrOfSegments();
	printf("nj=%d, ns=%d\n",nj,ns);
	JntArray jointpositions = JntArray(nj); 
	JntArray qz(nj);
	JntArray q_last(nj);
	Frame cartpos;
	//used for time statistic
	int sc_clk_tck = sysconf(_SC_CLK_TCK);
	struct tms begin_tms,end_tms;
	clock_t begin,end;
	bool kinematics_status;
	/*
	for(unsigned int i=0;i<nj;i++){ 
		float myinput; 
		printf("Enter the initial position of joint %i: ",i);
		scanf("%e",&myinput); 
		qz(i)=(double)myinput;
	} 
*/
	for(unsigned int i=0;i<nj;i++){

		qz(i)=0;
		q_last(i) = 0;
	}

	//inverse kinematics
	//initialize the solver
	double eps = 1e-5;
	double eps_joints = 1e-15;
	int maxiter = 500;
	ChainIkSolverPos_LMA iksolver(chain,eps,maxiter,eps_joints);
	// construct the destination frame
	Vector vec(0.5963,-0.1501,0.3144);
	Rotation rot(1,0,0,0,1,0,0,0,1);
	Frame destT(rot,vec);
/*
	for(unsigned int i = 0; i < 6;i++)
		qz(i) = 0;
	*/

	begin = clock();
	kinematics_status = iksolver.CartToJnt(q_last, destT, jointpositions);

	end = clock();
	if(kinematics_status >= 0){
		for(unsigned int i = 0; i < 7;i++){
			std::cout << jointpositions(i) << std::endl;
			std_msgs::Float64 num;
			num.data = jointpositions(i);
			joint_state.position[i] = jointpositions(i);
			//joint_angle_publisher[i].publish(num);
		}
		
		printf("ik solver succeed!\r\n");


	}
	else
		printf("ik solver failed!\r\n");

	printf("ik solver time costs: %lf\r\n",(end - begin)/(double)CLOCKS_PER_SEC);
	joint_state.header.stamp = ros::Time::now();
	jointstates_publisher.publish(joint_state);

	ros::spinOnce();
	//foward kinematic
	kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
	if(kinematics_status>=0){
		std::cout << cartpos << std::endl;
		printf("%s \n","Fkine Success, thanks KDL!");
	}
	else{
		printf("%s \n","Error:could not calculate forward kinematics : ");
	}
	while(ros::ok()){
		if(!new_target){
			ros::spinOnce();
			continue;
		}
		vec.x(target.linear.x);
		vec.y(target.linear.y);
		vec.z(target.linear.z);

		//rot.RPY(target.angular.x, target.angular.y, target.angular.z);
		//rot.EulerZYX(target.angular.z, target.angular.y, target.angular.x);
		rot = Rotation::RPY(target.angular.x, target.angular.y, target.angular.z);
		Frame targetT(rot,vec);
		double fr,fp,fy;
		double tx,ty,tz;
		rot.GetRPY(fr, fp, fy);
		rot.GetEulerZYX(tz, ty, tx);
		printf("rpy:%f, %f, %f\n", fr,fp,fy);
		printf("zyx:%f, %f, %f\n", tz,ty,tx);


		begin = clock();
		kinematics_status = iksolver.CartToJnt(qz, targetT, jointpositions);

		end = clock();
		if(kinematics_status >= 0){
			for(unsigned int i = 0; i < 7;i++){
				std::cout << jointpositions(i) << std::endl;
				std_msgs::Float64 num;
				num.data = jointpositions(i);
				q_last(i) = jointpositions(i);

				//joint_angle_publisher[i].publish(num);
				joint_state.position[i] = jointpositions(i);
			}
			joint_state.header.stamp = ros::Time::now();
			jointstates_publisher.publish(joint_state);
			printf("ik solver succeed!\r\n");


		}
		else
			printf("ik solver failed!\r\n");
		new_target = false;
		ros::spinOnce();
	}



}

