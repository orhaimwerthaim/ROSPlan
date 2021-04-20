#include "rosplan_action_interface/PickActionInterface.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_planning_system/PlanDispatch/EsterelPlanDispatcher.h"
#include "diagnostic_msgs/KeyValue.h"
#include <iostream>
#include "robotican_demos_upgrade/pick_unknown.h" 
#include <string>
using namespace std;

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

/* constructor */
	PickActionInterface::PickActionInterface(ros::NodeHandle &nh) {
		// perform setup
	}

	/* action dispatch callback */
	bool PickActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.

		// complete the action
		ROS_WARN("KCL: (%s) ---------------------------------------------------------------------------PickActionInterface Action completing.", msg->name.c_str());
ROS_WARN("KCL: PickActionInterface Parameters: robot:%s obj:%s location:%s'", msg->parameters[0].value.c_str(),msg->parameters[1].value.c_str(),msg->parameters[2].value.c_str());


		// ROS_INFO("KCL: parameters1:{key='%s',value='%s'} parameters2:{key='%s',value='%s'}", \
		// msg->parameters[0].key, msg->parameters[0].value, msg->parameters[1].key, msg->parameters[1].value);


if(useSimulationServices)
{
ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robotican_demos_upgrade::pick_unknown>("pick_unknown");
  robotican_demos_upgrade::pick_unknown srv;
  srv.request.robot = msg->parameters[0].value.c_str();
  srv.request.obj = msg->parameters[1].value.c_str();
  srv.request.discrete_location = msg->parameters[2].value.c_str();
  if (client.call(srv))
  {
    ROS_WARN("KCL: (%s) Service pick was called from ROSPlan action", msg->name.c_str());
  }
  else
  {
    ROS_ERROR("KCL: Failed to call service pick");
    return false;
  }
}








//rosservice call /rosplan_plan_dispatcher/cancel_dispatch
// ros::NodeHandle n;
//   ros::ServiceClient client = n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update");
//   rosplan_knowledge_msgs::KnowledgeUpdateService srv;
//   srv.request.update_type = 0;
//   srv.request.knowledge.knowledge_type = 1;
// srv.request.knowledge.attribute_name = "object_at";
// diagnostic_msgs::KeyValue keyVal1;
// keyVal1.key ="objects";
// keyVal1.value = "can";
// diagnostic_msgs::KeyValue keyVal2;
// keyVal1.key ="discrete_location";
// keyVal1.value = "outside_lab211";
// srv.request.knowledge.values.push_back(keyVal1);
// srv.request.knowledge.values.push_back(keyVal2);
//   client.call(srv);
//system("/home/or/catkin_ws_elevator-master/src/rosplan_experiment_pddl/replanning.bash");
ROS_WARN("KCL: (%s) ************************************************************************PickActionInterface Action completing.", msg->name.c_str());
// ros::ServiceClient client2 = n.serviceClient<KCL_rosplan::EsterelPlanDispatcher::cancelDispatchService>("/rosplan_plan_dispatcher/cancel_dispatch");		
// 		 KCL_rosplan::PlanDispatcher srv2;
// 		 client2.call(srv2);
		
		
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
		KCL_rosplan::PickActionInterface rpti(nh);

		rpti.runActionInterface();

		return 0;
	}