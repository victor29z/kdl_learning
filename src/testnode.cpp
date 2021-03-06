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

using namespace KDL; 
using namespace std; 

bool new_target = false;
geometry_msgs::Twist target;

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
	ros::Publisher joint_angle_publisher[6];
	joint_angle_publisher[0] = n.advertise<std_msgs::Float64>("joint1_position_controller/command", 1000);
	joint_angle_publisher[1] = n.advertise<std_msgs::Float64>("joint2_position_controller/command", 1000);
	joint_angle_publisher[2] = n.advertise<std_msgs::Float64>("joint3_position_controller/command", 1000);
	joint_angle_publisher[3] = n.advertise<std_msgs::Float64>("joint4_position_controller/command", 1000);
	joint_angle_publisher[4] = n.advertise<std_msgs::Float64>("joint5_position_controller/command", 1000);
	joint_angle_publisher[5] = n.advertise<std_msgs::Float64>("joint6_position_controller/command", 1000);

	ros::Subscriber tarpos_sub = n.subscribe("tarpos_pub", 1000, posmsgCallback);

	Tree my_tree; 
	kdl_parser::treeFromFile("/home/zl/catkin_ws/src/myur_description/urdf/ur5_robot_with_body_top_mount.urdf",my_tree);
	bool exit_value; 
	Chain chain; 
	exit_value = my_tree.getChain("world","wrist_1_link",chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	//ChainJntToJacSolver jac_solver = ChainJntToJacSolver(chain);
	ChainJntToJacSolver jac_solver(chain);
	unsigned int nj = chain.getNrOfJoints(); 
	unsigned int ns = chain.getNrOfSegments();
	printf("nj=%d, ns=%d\n",nj,ns);
	JntArray jointpositions = JntArray(nj); 
	JntArray qz(nj);
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
	kinematics_status = iksolver.CartToJnt(qz, destT, jointpositions);

	end = clock();
	if(kinematics_status >= 0){
		for(unsigned int i = 0; i < nj;i++){
			std::cout << jointpositions(i) << std::endl;
			std_msgs::Float64 num;
			num.data = jointpositions(i);

			joint_angle_publisher[i].publish(num);
		}

		printf("ik solver succeed!\r\n");


	}
	else
		printf("ik solver failed!\r\n");

	printf("ik solver time costs: %lf\r\n",(end - begin)/(double)CLOCKS_PER_SEC);
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
			for(unsigned int i = 0; i < nj;i++){
				std::cout << jointpositions(i) << std::endl;
				std_msgs::Float64 num;
				num.data = jointpositions(i);

				joint_angle_publisher[i].publish(num);
			}

			printf("ik solver succeed!\r\n");


		}
		else
			printf("ik solver failed!\r\n");
		new_target = false;
		ros::spinOnce();
	}


	/*
	//jacobian
	Jacobian jcb;

	kinematics_status = jac_solver.JntToJac(jointpositions,jcb,-1);
	if(kinematics_status>=0){
		printf("Jacobian Structure: %d x %d \r\n",jcb.columns(),jcb.rows());
		for(unsigned int i = 0;i < 6; i++){
			printf("[");
			for(unsigned int j = 0; j < 6; j++) printf("%f, ", jcb(i,j));
			printf("]\r\n");
		}
		printf("%s \n","Jacobian Success, thanks KDL!");
	}
	else{
		printf("%s \n","Error:could not calculate jacobian : ");
	}
	*/

}

