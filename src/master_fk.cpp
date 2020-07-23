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
	kdl_parser::treeFromFile("/home/robot/catkin_ws/src/srs-77dof/urdf/srs-77dof.urdf",my_tree); 
	bool exit_value; 
	Chain chain; 
	exit_value = my_tree.getChain("base_link","right_tcp",chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	//ChainJntToJacSolver jac_solver = ChainJntToJacSolver(chain);
	ChainJntToJacSolver jac_solver(chain);
	unsigned int nj = chain.getNrOfJoints(); 
	JntArray jointpositions = JntArray(nj); 
	JntArray qz(nj);
	Frame cartpos;
	//used for time statistic
	
	bool kinematics_status;
	
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

