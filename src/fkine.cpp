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

#include <stdio.h> 
#include <iostream> 
#include <sys/times.h>
#include <unistd.h>
#include <stdlib.h>

using namespace KDL; 
using namespace std; 
int main(int argc,char** argv){ 
	Tree my_tree; 
	kdl_parser::treeFromFile("/home/zl/catkin_ws/src/myur_description/urdf/ur5_backup.urdf",my_tree); 
	bool exit_value; 
	Chain chain; 
	exit_value = my_tree.getChain("base","tool0",chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	//ChainJntToJacSolver jac_solver = ChainJntToJacSolver(chain);
	ChainJntToJacSolver jac_solver(chain);
	unsigned int nj = chain.getNrOfJoints(); 
	JntArray jointpositions = JntArray(nj); 
	JntArray qz(nj);
	Frame cartpos;
	//used for time statistic
	int sc_clk_tck = sysconf(_SC_CLK_TCK);
	struct tms begin_tms,end_tms;
	clock_t begin,end;
	bool kinematics_status;
	for(unsigned int i=0;i<nj;i++){ 
		float myinput; 
		printf("Enter the initial position of joint %i: ",i);
		scanf("%e",&myinput); 
		qz(i)=(double)myinput;
	} 

	//inverse kinematics
	//initialize the solver
	double eps = 1e-5;
	double eps_joints = 1e-15;
	int maxiter = 500;
	ChainIkSolverPos_LMA iksolver(chain,eps,maxiter,eps_joints);
	// construct the destination frame
	Vector vec(0.5963,-0.1501,-0.0144);
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
		for(unsigned int i = 0; i < 6;i++)
			std::cout << jointpositions(i) << std::endl;
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

}

