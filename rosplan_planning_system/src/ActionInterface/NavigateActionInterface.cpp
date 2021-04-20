#include "rosplan_action_interface/NavigateActionInterface.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_planning_system/PlanDispatch/EsterelPlanDispatcher.h"
#include "diagnostic_msgs/KeyValue.h"
#include <iostream>
#include "robotican_demos_upgrade/robot_navigation.h" 
#include <string>
using namespace std;
 
/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

/* constructor */
	NavigateActionInterface::NavigateActionInterface(ros::NodeHandle &nh) {
		// perform setup
	}


	/* action dispatch callback */
	bool NavigateActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.

		// complete the action
		ROS_WARN("KCL: (%s) ---------------------------------------------------------------------------NavigateActionInterface Action completing.", msg->name.c_str());

ROS_WARN("KCL: NavigateActionInterface Parameters: robot:%s orig:%s dest:%s'", msg->parameters[0].value.c_str(),msg->parameters[1].value.c_str(),msg->parameters[2].value.c_str());

		// ROS_INFO("KCL: parameters1:{key='%s',value='%s'} parameters2:{key='%s',value='%s'}", \
		// msg->parameters[0].key, msg->parameters[0].value, msg->parameters[1].key, msg->parameters[1].value);

try {
    

if(useSimulationServices)
{
ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robotican_demos_upgrade::robot_navigation>("robot_navigation");
  robotican_demos_upgrade::robot_navigation srv;
 

  srv.request.nav_name =  msg->parameters[2].value.c_str();  
  srv.request.robot = msg->parameters[1].value.c_str();
  srv.request.discrete_location1 = msg->parameters[1].value.c_str();
  srv.request.discrete_location2 = msg->parameters[1].value.c_str();
  srv.request.floor = msg->parameters[1].value.c_str();
  if (client.call(srv))
  {
    ROS_WARN("KCL: (%s) Service robot_navigation was called from ROSPlan action", msg->name.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service robot_navigation");
    return false;
  }  
}
} catch (const std::exception& e) { // reference to the base of a polymorphic object
  std::string errorStr = e.what();
     ROS_WARN("ERROR: (%s)", (errorStr.c_str())); 
}
ROS_WARN("KCL: (%s) ************************************************************************NavigateActionInterface Action completing.", msg->name.c_str());
   //ros::Duration(60).sleep();
		return true;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_tutorial_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::NavigateActionInterface rpti(nh);

		rpti.runActionInterface();

		return 0;
	}