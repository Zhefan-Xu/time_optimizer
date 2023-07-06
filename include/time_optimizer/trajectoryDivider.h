/*
*	File: trajectoryDivider.h
*	---------------
*   dynamic exploration planner header file
*/

#ifndef TRAJECTORY_DIVIDER_H
#define TRAJECTORY_DIVIDER_H

#include <ros/ros.h>
#include <map_manager/dynamicMap.h>
#include <global_planner/KDTree.h>

namespace timeOptimizer{
	class trajDivider{
	private:
		ros::NodeHandle nh_;
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::vector<Eigen::Vector3d> trajectory_;
		std::vector<double> time_;
		int maxLengthIdx_ = 0;
		std::shared_ptr<KDTree::KDTree<3, int>> kdtree_;

		// parameter
		double maxLength_ = 7.0; // m
		double safeDist_ = 1.0; // m


		

	public:
		trajDivider(const ros::NodeHandle& nh);
		void setMap(const std::shared_ptr<mapManager::dynamicMap>& map);
		void setTrajectory(const std::vector<Eigen::Vector3d>& trajectory, const std::vector<double>& time);
	
		void run(std::vector<double>& splitTime);
		void findRange(Eigen::Vector3d& rangeMin, Eigen::Vector3d& rangeMax);
		void buildKDTree(const Eigen::Vector3d& rangeMin, const Eigen::Vector3d& rangeMax);
		void findNearestObstacles(std::vector<Eigen::Vector3d>& nearestObstacles, std::vector<bool>& mask);
	};
}

#endif