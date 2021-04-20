#include <ros/ros.h>
#include <vector>

#include "rosplan_action_interface/RPActionInterface.h"

#ifndef KCL_tutorial_10
#define KCL_tutorial_10

/**
 * This file defines an action interface created in tutorial 10.
 */
namespace KCL_rosplan {
bool robot_navigation;
	class NavigateActionInterface: public RPActionInterface
	{

	private:
	//	void updateFact(const std::string name, vector<string> &par_keys, vector<string> &par_values, bool isAdd);
	public:

		/* constructor */
		NavigateActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif

