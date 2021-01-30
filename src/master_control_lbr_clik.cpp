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
#include <kdl/chainiksolvervel_pinv.hpp>

#include <stdio.h> 
#include <iostream> 
#include <fstream>
#include <sys/times.h>
#include <unistd.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <eigen_conversions/eigen_kdl.h>
#include <math.h>

#include <Eigen/Dense>

#include<time.h>
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <arpa/inet.h>

#define NO_OF_JOINTS 7

using namespace KDL; 
using namespace std; 
using namespace Eigen;

bool new_target = false;
geometry_msgs::Twist target;
sensor_msgs::JointState joint_state;
float master_joint_pos[7];
JntArray xmate_joint_positions = JntArray(7); 
JntArray xmate_joint_positions_clik = JntArray(7);
JntArray master_joint_positions = JntArray(7); 
int master_jaw_state = -1;

const char* j_name_list[]={
"joint_a1",
"joint_a2",
"joint_a3",
"joint_a4",
"joint_a5",
"joint_a6",
"joint_a7"
};

Tree xmate_tree,master_tree;
/* main chain*/
Chain chain,master_chain, xmate_chain;
/* sub chain*/
Chain master_subchain, xmate_subchain;
/* log file stream*/
ofstream fout;

#define TO_ROBOSUITE_PORT 9170
unsigned int slave_port = TO_ROBOSUITE_PORT;
char slave_addr[20] = "127.0.0.1";

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


void master_data_receive_Callback(const sensor_msgs::JointStateConstPtr& msg)
{
	
	int i;
	new_target = true;
	for(i = 0; i < 7; i++)
		master_joint_positions(i)=msg->position[i];
	master_jaw_state = msg->position[14];



  //printf("I heard: [%f] [%f] [%f] [%f] [%f] [%f] [%f]\r\n",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6]);
 // ROS_INFO("I heard: [%f] [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6]);
 
}

void print_frame(Frame frame_to_print){

	printf("position: %.2f, %.2f, %.2f\n",frame_to_print.p.data[0],frame_to_print.p.data[1],frame_to_print.p.data[2]);
	double target_rpy[3];
	frame_to_print.M.GetRPY(target_rpy[0],target_rpy[1],target_rpy[2]);
	printf("angle: %.2f, %.2f, %.2f\n", target_rpy[0] ,target_rpy[1],target_rpy[2]);


}

void get_6x1_vector_from_frame(Frame frame, std::vector<double>& vec){

	frame.M.GetRPY(vec[0],vec[1],vec[2]);
	vec[3] = frame.p.data[0];
	vec[4] = frame.p.data[1];
	vec[5] = frame.p.data[2];

}

int right_pinv(Jacobian jcb, Eigen::MatrixXd &dest_mat, double tolerance = 1.e-6)
{


	Eigen::MatrixXd source_mat;
	source_mat = jcb.data * jcb.data.transpose();
	//fprintf(stderr, "source matrix:\n");
	//std::cout << source_mat << std::endl;

	//fprintf(stderr, "\nEigen implement pseudoinverse:\n");
	auto svd = source_mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

	const auto &singularValues = svd.singularValues();
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(source_mat.cols(), source_mat.rows());
	singularValuesInv.setZero();
	//double  pinvtoler = 1.e-6; // choose your tolerance wisely
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > tolerance)
			singularValuesInv(i, i) = 1.0f / singularValues(i);
		else
			singularValuesInv(i, i) = 0.f;
	}

	Eigen::MatrixXd pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
	//std::cout << pinvmat << std::endl;
	dest_mat = jcb.data.transpose() * pinvmat;

	return 0;
}


int pinv(Jacobian jcb, Eigen::MatrixXd &dest_mat, double tolerance = 1.e-6)
{
/*
	std::vector<std::vector<float>> vec{ { 0.68f, 0.597f, -0.211f },
					{ 0.823f, 0.566f, -0.605f } };
	const int rows{ 2 }, cols{ 3 };

	std::vector<float> vec_;
	for (int i = 0; i < rows; ++i) {
		vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());
	}
	Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(source_mat.data(), 6, 7);
*/

	//Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> source_mat(jcb.data, 6, 7);
	Eigen::MatrixXd source_mat;
	source_mat = jcb.data;
	//fprintf(stderr, "source matrix:\n");
	//std::cout << source_mat << std::endl;

	//fprintf(stderr, "\nEigen implement pseudoinverse:\n");
	auto svd = source_mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

	const auto &singularValues = svd.singularValues();
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(source_mat.cols(), source_mat.rows());
	singularValuesInv.setZero();
	//double  pinvtoler = 1.e-6; // choose your tolerance wisely
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > tolerance)
			singularValuesInv(i, i) = 1.0f / singularValues(i);
		else
			singularValuesInv(i, i) = 0.f;
	}

	Eigen::MatrixXd pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
	//std::cout << pinvmat << std::endl;
	dest_mat = pinvmat;

	return 0;
}

int clik_solver(JntArray master_data, Frame cartisian_target, JntArray slave_current, JntArray& slave_data){

	slave_data.resize(7);

	JntArray zero_position(7);
	JntArray qold(7),qout(7);
	/* used to get joint space velocity*/
	JntArray qdot_out(7);
	/* save temporary frame of slave end-effector*/
	Frame slave_temp_pos;
	/* 6-D differential between target and slave end-effector*/
	Twist diffet;
	/* slave fk solver*/
	ChainFkSolverPos_recursive slave_fksolver = ChainFkSolverPos_recursive(xmate_chain);
	Jacobian jcb;
	/* slave jacobian solver*/
	ChainJntToJacSolver jac_solver = ChainJntToJacSolver(xmate_chain);
	/* use kdl velocity solver to get joint space velocity from Twist*/
	ChainIkSolverVel_pinv vel_solver = ChainIkSolverVel_pinv (xmate_chain);
	int i,step;
	double norm_rot, norm_vel;
	/*initialization*/
	for(i = 1; i < 7; i++)
		zero_position(i) = 0;


	qout = slave_current;
	qold = slave_current;
	/* adjust end-effector of master and slave to the same direction*/
	//cartisian_target.M.DoRotZ(M_PI_2);
/*
	printf("target position: %.2f, %.2f, %.2f\n",cartisian_target.p.data[0],cartisian_target.p.data[1],cartisian_target.p.data[2]);
	double target_rpy[3];
	cartisian_target.M.GetRPY(target_rpy[0],target_rpy[1],target_rpy[2]);
	printf("target angle: %.2f, %.2f, %.2f\n", target_rpy[0] ,target_rpy[1],target_rpy[2]);
*/
	print_frame(cartisian_target);
	/* clear iteration steps*/

	step = 0;
	norm_rot = 0;
	norm_vel = 0;
	do{
		printf("---------------step%d-----------------\n",step);
		/* slave forward kinematic*/
		slave_fksolver.JntToCart(qout, slave_temp_pos);
		print_frame(slave_temp_pos);
		/* caculate differential between target and current slave end-effector*/
		diffet = diff( slave_temp_pos, cartisian_target);
		/*
		cout<<"twist:\n"
				<<diffet.vel.data[0]<<","
				<<diffet.vel.data[1]<<","
				<<diffet.vel.data[2]<<","
				<<diffet.rot.data[0]<<","
				<<diffet.rot.data[1]<<","
				<<diffet.rot.data[2]<<endl;
		*/

		/* caculate slave jacobian*/
		jcb.resize(7);
		jac_solver.JntToJac(qout,jcb);
		vel_solver.CartToJnt(qold,diffet,qdot_out);


		/* use Kp = 0.5 as iteration rate*/
		qdot_out.data *= 0.5;
		qout.data = qold.data + qdot_out.data;
		qold = qout;
		norm_rot = diffet.rot.Norm();
		norm_vel = diffet.vel.Norm();
		/* if convergence cannot be reached within 100 step then return -1*/
		if(step++ > 100) {
			printf( "ik solve failed\n");
			return -1;
		}

	}while(norm_rot > 1e-3 || norm_vel > 1e-3);
	slave_data = qout;
	printf( "clik solve succeed! step=%d\n",step);
	/* if find a solution then return the iteration steps*/
	return step;




}

int clik_solver_pinv(JntArray master_data, Frame cartisian_target, JntArray slave_current, JntArray& slave_data){

	slave_data.resize(7);

	JntArray zero_position(7);
	JntArray qold(7),qout(7);
	/* used to get joint space velocity*/
	JntArray qdot_out(7);
	/* save temporary frame of slave end-effector*/
	Frame slave_ee_temp_pos;
	/* save constraint frame of master device*/
	Frame constraint_frame;
	Frame slave_constraint_temp_pos;
	/* 6-D differential between target and slave end-effector*/
	Twist ee_twist;
	/* 6-D differential between constraint frame of master and slave*/
	Twist constraint_twist;

	/* slave fk solver*/
	ChainFkSolverPos_recursive slave_fksolver = ChainFkSolverPos_recursive(xmate_chain);
	Jacobian jcb_ee, jcb_constraint;
	/* slave end-effector jacobian solver*/
	ChainJntToJacSolver ee_jac_solver = ChainJntToJacSolver(xmate_chain);
	/* slave end-effector jacobian solver*/
	ChainJntToJacSolver constraint_jac_solver = ChainJntToJacSolver(xmate_subchain);

	/* use kdl velocity solver to get joint space velocity from Twist*/
	ChainIkSolverVel_pinv vel_solver = ChainIkSolverVel_pinv (xmate_chain);
	/* master sub chain fk solver*/
	ChainFkSolverPos_recursive master_subchain_fksolver = ChainFkSolverPos_recursive(master_subchain);
	/* slave sub chain fk solver*/
	ChainFkSolverPos_recursive slave_subchain_fksolver = ChainFkSolverPos_recursive(xmate_subchain);

	/* 6x1 end-effector and constraint vector*/
	std::vector<double> vec_ee(6), vec_constraint(6);
	int i,step;
	double norm_rot, norm_vel;
	/*initialization*/
	for(i = 1; i < 7; i++)
		zero_position(i) = 0;


	qout = slave_current;
	qold = slave_current;
	/* adjust end-effector of master and slave to the same direction*/
	//cartisian_target.M.DoRotZ(M_PI_2);
/*
	printf("target position: %.2f, %.2f, %.2f\n",cartisian_target.p.data[0],cartisian_target.p.data[1],cartisian_target.p.data[2]);
	double target_rpy[3];
	cartisian_target.M.GetRPY(target_rpy[0],target_rpy[1],target_rpy[2]);
	printf("target angle: %.2f, %.2f, %.2f\n", target_rpy[0] ,target_rpy[1],target_rpy[2]);
*/
	cartisian_target.M.DoRotY(-M_PI_2);
	cartisian_target.M.DoRotX(-M_PI_2);
	//cartisian_target.M.DoRotY(-M_PI_2);
	/* mulply by 1.2 to enlarge slave work space*/
	cartisian_target.p.data[0] = cartisian_target.p.data[0] * 1.2;
	cartisian_target.p.data[1] = cartisian_target.p.data[1] * 1.2;
	cartisian_target.p.data[2] = cartisian_target.p.data[2] * 1.2;


	print_frame(cartisian_target);
	get_6x1_vector_from_frame(cartisian_target, vec_ee);
	/* clear iteration steps*/

	step = 0;
	norm_rot = 0;
	norm_vel = 0;
	/* jacobian must be difined a size before used*/
	jcb_ee.resize(7);
	jcb_constraint.resize(4);
	/* caculate the target constraint from master device*/
	JntArray master_subchain_data;

	master_subchain_data.resize(3);
	master_subchain_data.data = master_data.data.head(3);
	//for(i = 0; i <3; i++)
	//	master_subchain_data(i) = master_data(i);
	master_subchain_fksolver.JntToCart(master_subchain_data, constraint_frame);
	/* transform the target constraint frame into direction consistent with slave link4*/
	/* rotate pi/2 about Y, then rotate pi about Z*/
	constraint_frame.M.DoRotY(M_PI_2);
	constraint_frame.M.DoRotZ(M_PI);
	get_6x1_vector_from_frame(constraint_frame, vec_constraint);
	//printf("constraint frame\n");
	//print_frame(constraint_frame);
	fout 	<< vec_ee[0]<<" "
			<< vec_ee[1]<<" "
			<< vec_ee[2]<<" "
			<< vec_ee[3]<<" "
			<< vec_ee[4]<<" "
			<< vec_ee[5]<<", "
			<< vec_constraint[0]<<" "
			<< vec_constraint[1]<<" "
			<< vec_constraint[2]<<" "
			<< vec_constraint[3]<<" "
			<< vec_constraint[4]<<" "
			<< vec_constraint[5]<<", ";

	do{
		printf("---------------step%d-----------------\n",step);
		/* slave forward kinematic*/
		slave_fksolver.JntToCart(qout, slave_ee_temp_pos);
		/* slave constraint sub chain fk*/
		JntArray slave_constraint_joint_data;
		slave_constraint_joint_data.resize(4);
		slave_constraint_joint_data.data = qout.data.head(4);
		slave_subchain_fksolver.JntToCart(slave_constraint_joint_data, slave_constraint_temp_pos);
		//cout<<"slave ee:"<<endl;
		//print_frame(slave_ee_temp_pos);
		//cout<<"slave link4:"<<endl;
		//print_frame(slave_constraint_temp_pos);
		/* calculate differential between target and current slave end-effector*/
		ee_twist = diff(slave_ee_temp_pos, cartisian_target);
		/* calculate differential of constraint frames*/
		constraint_twist = diff(slave_constraint_temp_pos, constraint_frame);
		/* clear velocity, just use rotation for constraint */
		SetToZero( constraint_twist.vel);
		//cout<<"constraint_twist:"<<constraint_twist<<endl;
		/*

		cout<<"qout:\n"
				<<qout.data<<endl;
	 			*/
		/* caculate slave jacobian*/

		ee_jac_solver.JntToJac(qout,jcb_ee);
		constraint_jac_solver.JntToJac(slave_constraint_joint_data,jcb_constraint);
		/* caculate pinv of jacobian*/
		Eigen::MatrixXd jr_ee, jr_constraint;
		//cout<< "jacobian structure:"<<j_pinv.rows()<<"x"<<j_pinv.cols()<<endl;
		//cout<< "jacobian structure:"<<jcb.data.rows()<<"x"<<jcb.data.cols()<<endl;
		pinv(jcb_ee, jr_ee);
		right_pinv(jcb_constraint, jr_constraint);
		/*
		cout<<"jr_constraint:\n"
								<<jr_constraint<<endl;
		*/

		/* use Kp = 0.5 as iteration rate*/

		Eigen::Matrix<double, 6, 1> ee_twist_vec, constraint_twist_vec;
		tf::twistKDLToEigen(ee_twist , ee_twist_vec);
		tf::twistKDLToEigen(constraint_twist , constraint_twist_vec);
		/*
		cout<<"twist_vec:\n"
						<<ee_twist_vec<<endl;
		*/

		/* small increasement on q vector*/
		Eigen::Matrix<double, 7, 1> qinc;
		qinc = jr_ee * ee_twist_vec ;
		//qinc.operator *(0.01);
		qinc*= 0.5;
		/* small increasement on null space q0*/
		Eigen::Matrix<double, 4, 1> q0inc;
		/*          4x6       x      6x1    */
		q0inc = jr_constraint * constraint_twist_vec ;
		q0inc*= 0.5;
		/*
		cout<<"q0inc:\n"
			<<q0inc<<endl;
		*/
		Eigen::Matrix<double, 7, 7> null_space_projector;


		null_space_projector = Eigen::MatrixXd::Identity(7,7) - jr_ee * jcb_ee.data;
		/*
		cout<<"null space projector:\n"
			<<null_space_projector<<endl;
		*/
		/* q0_inc augment to 7x1*/
		Eigen::Matrix<double, 7, 1> q0inc_aug;
		q0inc_aug.setZero();
		q0inc_aug.head(4) = q0inc;
		/*
		cout<<"q0inc_aug:\n"
			<<q0inc_aug<<endl;
		*/
		/*
		cout<<"q increasement:\n"
			<<qinc<<endl;
		*/
		qout.data = qold.data + qinc + null_space_projector * q0inc_aug;
		//qout.data = qold.data + qinc;
		/*
		cout<<"qout_updated:\n"
						<<qout.data<<endl;
		*/
		qold = qout;

		norm_rot = ee_twist.rot.Norm();
		norm_vel = ee_twist.vel.Norm();
		/* if convergence cannot be reached within 500 step then return -1*/
		if(step++ > 500) {
			printf( "ik solve failed\n");
			return -1;
		}

	}while(norm_rot > 1e-3 || norm_vel > 1e-3);
	for(i = 0; i < 7; i++){

		int scalar = qout(i) / M_PI / 2;

		qout(i) -= (scalar) * M_PI * 2;

		if(qout(i) > M_PI) qout(i) -= M_PI * 2;
		if(qout(i) < -M_PI) qout(i) += M_PI * 2;


	}

	slave_data = qout;
	printf( "clik solve succeed! step=%d\n",step);
	/* if find a solution then return the iteration steps*/
	return step;




}
int main(int argc,char** argv){
	ros::init(argc, argv, "ik_solver");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	/* setup publisher to control slave arm*/
	ros::Publisher jointstates_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
   
	/* setup subscriber to receive master joint information*/
	ros::Subscriber master_data_subscriber = n.subscribe("master_joint_states", 1000, master_data_receive_Callback);
	/* get systime */
	time_t now = time(NULL);
	tm* tm_t = localtime(&now);
	std::stringstream file_name;
	file_name <<"/home/robot/log/clik_log_"<<tm_t->tm_year + 1900  << tm_t->tm_mon + 1  << tm_t->tm_mday
		<< tm_t->tm_hour << tm_t->tm_min << tm_t->tm_sec;
    //std::cout << file_name.str();

	/* create log file*/
	fout.open(file_name.str());



	
	Joint_State_Msg_Initialize(NO_OF_JOINTS,(char**)j_name_list);



	/* load master and slave urdf model*/

	kdl_parser::treeFromFile("/home/robot/catkin_ws/src/kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf",xmate_tree);
	kdl_parser::treeFromFile("/home/robot/catkin_ws/src/srs-77dof/urdf/srs-77dof.urdf",master_tree); 
	bool exit_value; 
	/* get kinematic chain of master and slave*/

	xmate_tree.getChain("base_link","tool0",xmate_chain);
	master_tree.getChain("base_link","right_tcp",master_chain);

	/* get sub chain for constraints*/
	xmate_tree.getChain("base_link","link4",xmate_subchain);
	master_tree.getChain("base_link","link-r3",master_subchain);

	/* create forward kinematic solver of master and slave*/
	ChainFkSolverPos_recursive slave_fksolver = ChainFkSolverPos_recursive(xmate_chain);
	ChainFkSolverPos_recursive master_fksolver = ChainFkSolverPos_recursive(master_chain);
 
	unsigned int xmate_nj = xmate_chain.getNrOfJoints(); 
	unsigned int xmate_ns = xmate_chain.getNrOfSegments();
	printf("xmate_nj=%d, ns=%d\n",xmate_nj,xmate_ns);

	JntArray qz(xmate_nj);
	JntArray q_last(xmate_nj);
	Frame cartpos,master_tcp_pos;
	//used for time statistic
	int sc_clk_tck = sysconf(_SC_CLK_TCK);
	struct tms begin_tms,end_tms;
	clock_t begin,end;
	bool kinematics_status;
	int ik_status;
	/* used to count serial number of records*/
	unsigned long int sn = 0;



	for(unsigned int i=0;i<xmate_nj;i++){

		qz(i)=0;
		q_last(i) = 0;
	}

	/*TO slave UDP socket init*/
	int socketudp_toslave;
	struct sockaddr_in addr_toslave;
	//bzero(&addr_toslave, sizeof(addr_toslave));
	addr_toslave.sin_family = AF_INET;
	//addr_toslave.sin_addr = (in_addr_t)inet_addr(slave_addr);
	addr_toslave.sin_addr.s_addr = (in_addr_t)inet_addr(slave_addr);
	addr_toslave.sin_port = htons(TO_ROBOSUITE_PORT);

	socketudp_toslave = socket(AF_INET, SOCK_DGRAM, 0);
	socklen_t addr_udp_len = sizeof(addr_toslave);

	// for ik test only
	/*

	master_fksolver.JntToCart(qz,master_tcp_pos);
	//clik_solver_pinv(qz,master_tcp_pos, xmate_joint_positions_clik);
	clik_solver_pinv(qz,master_tcp_pos,xmate_joint_positions_clik, xmate_joint_positions_clik);

return 0;

*/
	stringstream out_test;



	while(ros::ok()){
		if(!new_target){
			ros::spinOnce();
			continue;
		}


		/*  solve master TCP position*/
		master_fksolver.JntToCart(master_joint_positions,master_tcp_pos);
		/* save master joint position and target position*/

		//fout << sn++ <<", "<< master_joint_positions.data.transpose() << ", " << master_tcp_pos.M <<", "<< master_tcp_pos.p<<", "  ;

        //begin = clock();
		/* get inverse kinematics of xmate*/
		fout << sn++ <<", ";
        ik_status = clik_solver_pinv(master_joint_positions,master_tcp_pos,xmate_joint_positions_clik, xmate_joint_positions_clik);
		//kinematics_status = iksolver.CartToJnt(qz, master_tcp_pos, xmate_joint_positions);

        fout << master_joint_positions.data.transpose() << ", " << xmate_joint_positions_clik.data.transpose() << endl;
        fout.flush();
        /*
        cout<<"***************record start****************"<<endl;
		cout<<out_test.str();
		cout<<"xxxxxxxxxxxxxxxrecord end  xxxxxxxxxxxxxxxx"<<endl;
		cout.flush();
		*/
		//end = clock();
		/* publish ik result*/
        float joint_data_to_robosuite[7];
		if(ik_status >= 0){
			for(unsigned int i = 0; i < 7;i++){
				//std::cout << xmate_joint_positions_clik(i) << std::endl;

				//joint_state.position[i] = xmate_joint_positions(i);
				joint_state.position[i] = xmate_joint_positions_clik(i);
				joint_data_to_robosuite[i] = xmate_joint_positions_clik(i);

			}
			joint_state.header.stamp = ros::Time::now();
			jointstates_publisher.publish(joint_state);
			//printf("kdl ik solver succeed!\r\n");
//			cout<<"output data:"
//				<<joint_data_to_robosuite[0]<<","
//				<<joint_data_to_robosuite[1]<<","
//				<<joint_data_to_robosuite[2]<<","
//				<<joint_data_to_robosuite[3]<<","
//				<<joint_data_to_robosuite[4]<<","
//				<<joint_data_to_robosuite[5]<<","
//				<<joint_data_to_robosuite[6]<<endl;
			char udp_buffer[256]={0};

			sprintf(udp_buffer,"%f %f %f %f %f %f %f %f",
					joint_data_to_robosuite[0],
					joint_data_to_robosuite[1],
					joint_data_to_robosuite[2],
					joint_data_to_robosuite[3],
					joint_data_to_robosuite[4],
					joint_data_to_robosuite[5],
					joint_data_to_robosuite[6],
					(float)master_jaw_state);
			printf("%s\n",udp_buffer);
			sendto(socketudp_toslave,(char*)(udp_buffer), sizeof(udp_buffer),0,(struct sockaddr*)&addr_toslave,addr_udp_len);


		}

		new_target = false;
		ros::spinOnce();
	}


}

