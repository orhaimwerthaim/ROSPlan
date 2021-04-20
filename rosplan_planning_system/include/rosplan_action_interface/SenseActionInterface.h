#include <ros/ros.h>
#include <vector>

#include "rosplan_action_interface/NoKbUpdateActionInterface.h"

#ifndef KCL_tutorial_10
#define KCL_tutorial_10

/**
 * This file defines an action interface created in tutorial 10.
 */
namespace KCL_rosplan {

	class SenseActionInterface: public NoKbUpdateActionInterface
	{

	private:
	//	void addFact(const std::string name, const std::string par1, const std::string par2);
	public:

		/* constructor */
		SenseActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif

