/*
*	File: trajectoryDivider.h
*	---------------
*   dynamic exploration planner header file
*/

#ifndef TRAJECTORY_DIVIDER_H
#define TRAJECTORY_DIVIDER_H

#include <ros/ros.h>

namespace timeOptimizer{
	class trajDivider{
	private:
		ros::NodeHandle nh_;

	public:
		trajDivider(const ros::NodeHandle& nh);
	};
}

#endif