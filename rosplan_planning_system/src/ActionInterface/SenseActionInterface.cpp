#include "rosplan_action_interface/SenseActionInterface.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_planning_system/PlanDispatch/EsterelPlanDispatcher.h"
#include "diagnostic_msgs/KeyValue.h"
#include "robotican_demos_upgrade/sense_object.h" 
#include <iostream>
using namespace std;

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

	/* constructor */
	SenseActionInterface::SenseActionInterface(ros::NodeHandle &nh) {
		// perform setup
	}

	/* action dispatch callback */
	bool SenseActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
   bool atCorridor = ((msg->parameters[2].value).find("corridor") != std::string::npos);
   
    ROS_WARN("KCL: (%s) ************************************************************************SenseActionInterface  Action start.", msg->name.c_str());
ROS_WARN("KCL: SenseActionInterface Parameters: robot:%s obj:%s location:%s'", msg->parameters[0].value.c_str(),msg->parameters[1].value.c_str(),msg->parameters[2].value.c_str());

bool sensed_can = false;
ros::NodeHandle n;
if(useSimulationServices)
{
  ros::ServiceClient client = n.serviceClient<robotican_demos_upgrade::sense_object>("sense_object");
  robotican_demos_upgrade::sense_object srv;
  srv.request.robot = msg->parameters[0].value.c_str();
  srv.request.obj = msg->parameters[1].value.c_str();
  srv.request.discrete_location = msg->parameters[2].value.c_str();
  
  
  if (client.call(srv))
  {
    sensed_can =  srv.response.response.find("true") != std::string::npos; 
    ROS_WARN("KCL: Sensing action result is: (%s)", (sensed_can ? "can observed!" : "can was NOT observed!"));
  }
  else
  {
    ROS_ERROR("KCL: Failed to call service sense_object");
    return false;
  }
}
else
{
  sensed_can = !atCorridor;
}
srand (time(NULL));
int ran = rand() % 100;
bool noise = ran < 30;
sensed_can = noise ? !sensed_can : sensed_can;
ROS_WARN("rand is '%d', when it is below 30 we add noise",ran);
if(noise)
{ROS_WARN("KCL: Noise was added so we changed the observation to 'can was%s sensed!'", (sensed_can ? "" : " NOT"));
}
//update KB

//create parameters
const int REMOVE_FACT = 2;
const int ADD_FACT = 0;
const int TYPE_KNOWLEDGE = 1;
diagnostic_msgs::KeyValue par_can;
	par_can.key ="o";//"objects";
	par_can.value = "can";
  	diagnostic_msgs::KeyValue parCorridor;
	parCorridor.key ="discrete_location";
	parCorridor.value = "corridor";
  diagnostic_msgs::KeyValue parOutsideLab211;
	parOutsideLab211.key ="discrete_location";
	parOutsideLab211.value = "outside_lab211";


ros::ServiceClient clientKbUpdate = n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update");
		rosplan_knowledge_msgs::KnowledgeUpdateService srvUpdate;
    

 
  if(!sensed_can)
  {
diagnostic_msgs::KeyValue addPossibleLocationPar = atCorridor ? parOutsideLab211 : parCorridor;
diagnostic_msgs::KeyValue removePossibleLocationPar = atCorridor ? parCorridor : parOutsideLab211;
 
  srvUpdate.request.update_type = REMOVE_FACT;
  srvUpdate.request.knowledge.knowledge_type = TYPE_KNOWLEDGE;
  srvUpdate.request.knowledge.attribute_name = "possible_location";
	srvUpdate.request.knowledge.values.push_back(par_can);
  srvUpdate.request.knowledge.values.push_back(removePossibleLocationPar);
 clientKbUpdate.call(srvUpdate);

  srvUpdate.request.update_type = ADD_FACT;
  srvUpdate.request.knowledge.knowledge_type = TYPE_KNOWLEDGE;
  srvUpdate.request.knowledge.attribute_name = "possible_location";
	srvUpdate.request.knowledge.values.clear();
  srvUpdate.request.knowledge.values.push_back(par_can);
  srvUpdate.request.knowledge.values.push_back(addPossibleLocationPar);
 clientKbUpdate.call(srvUpdate);
  }
  else
  {
    diagnostic_msgs::KeyValue addObjectAtPar = atCorridor ? parCorridor : parOutsideLab211;
srvUpdate.request.update_type = ADD_FACT;
  srvUpdate.request.knowledge.knowledge_type = TYPE_KNOWLEDGE;
  srvUpdate.request.knowledge.attribute_name = "object_at";
	srvUpdate.request.knowledge.values.clear();
  srvUpdate.request.knowledge.values.push_back(par_can);
  srvUpdate.request.knowledge.values.push_back(addObjectAtPar);
 clientKbUpdate.call(srvUpdate);
  }
 
// ros::ServiceClient client2 = n.serviceClient<KCL_rosplan::EsterelPlanDispatcher::cancelDispatchService>("/rosplan_plan_dispatcher/cancel_dispatch");		
ROS_WARN("KCL: (%s) ||||||||||||||||||||||||||||||||||||||||||||returned sensed_can=%s   (if 'false' will replan).", msg->name.c_str(), sensed_can ? "true" : "false");
ROS_WARN("KCL: (%s) ************************************************************************SenseActionInterface  Action completing.", msg->name.c_str());
		return sensed_can;
	}

// void addFact(const std::string name, const std::string par1, const std::string par2)
// {
// 	//add object_at can outside_lab211
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
// keyVal2.key ="discrete_location";
// keyVal2.value = "outside_lab211";//"outside_lab211";

// srv.request.knowledge.values.push_back(keyVal1);
// srv.request.knowledge.values.push_back(keyVal2);
// //ROS_INFO("KCL: (%s) ---------Adding by service (object_at can outside_lab211).", msg->name.c_str());
// client.call(srv);

// //remove object_at can corridor 
//   ros::ServiceClient client2 = n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update"); 
//   rosplan_knowledge_msgs::KnowledgeUpdateService srv2;
//   srv2.request.update_type = 2;
//   srv2.request.knowledge.knowledge_type = 1;
// srv2.request.knowledge.attribute_name = "object_at";
// diagnostic_msgs::KeyValue keyVal11;
// keyVal11.key ="objects";
// keyVal11.value = "can";
// diagnostic_msgs::KeyValue keyVal22;
// keyVal22.key ="discrete_location";
// keyVal22.value = "corridor";//"outside_lab211";
// srv2.request.knowledge.values.push_back(keyVal11);
// srv2.request.knowledge.values.push_back(keyVal22);
// //ROS_INFO("KCL: (%s) ---------Removing by service (object_at can corridor).", msg->name.c_str());
// client2.call(srv2);

// //remove possible_location can corridor   
// srv2.request.knowledge.attribute_name = "possible_location"; 
// keyVal11.key ="objects";
// keyVal11.value = "can"; 
// keyVal22.key ="discrete_location";
// keyVal22.value = "corridor";//"outside_lab211";
// //srv2.request.knowledge.values.push_back(keyVal11);
// //srv2.request.knowledge.values.push_back(keyVal22);
// //ROS_INFO("KCL: (%s) ---------Removing by service (object_at can corridor).", msg->name.c_str());
// client2.call(srv2);



// //system("/home/or/catkin_ws_elevator-master/src/rosplan_experiment_pddl/replanning.bash");
// }

	
} // close namespace

int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_tutorial_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::SenseActionInterface rpti(nh);

		rpti.runActionInterface();

		return 0;
	}
