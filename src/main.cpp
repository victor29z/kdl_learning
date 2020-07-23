#include <kdl/kdl.hpp> 
#include <kdl/chain.hpp> 
#include <kdl/tree.hpp> 
#include <kdl/segment.hpp> 
#include <kdl/chainfksolver.hpp> 
#include <kdl_parser/kdl_parser.hpp> 
#include <kdl/chainfksolverpos_recursive.hpp> 
#include <kdl/frames_io.hpp> 
#include <stdio.h> 
#include <iostream> 
using namespace KDL; 
using namespace std; 
int main(int argc,char** argv){ 
	Tree my_tree; 
	kdl_parser::treeFromFile("/home/file/catkin_ws/src/KDL_Test/src/ur3_robot.urdf",my_tree); 
	bool exit_value; 
	Chain chain; 
	exit_value = my_tree.getChain("base","tool0",chain); 
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain); 
	unsigned int nj = chain.getNrOfJoints(); JntArray jointpositions = JntArray(nj); 
	for(unsigned int i=0;i<nj;i++){ 
		float myinput; 
		printf("Enter the position of joint %i: ",i); 
		scanf("%e",&myinput); 
		jointpositions(i)=(double)myinput; 
	} 
	Frame cartpos;
	 
	bool kinematics_status; 
	kinematics_status = fksolver.JntToCart(jointpositions,cartpos); 
	if(kinematics_status>=0){ std::cout << cartpos << std::endl; 
		printf("%s \n","Success, thanks KDL!"); 
	} 
	else{ 
		printf("%s \n","Error:could not calculate forward kinematics : "); 
	} 

}

