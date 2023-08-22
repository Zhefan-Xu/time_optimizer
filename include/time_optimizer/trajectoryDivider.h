/*
*	File: trajectoryDivider.h
*	---------------
*   trajectory divider header file
*/

#ifndef TRAJECTORY_DIVIDER_H
#define TRAJECTORY_DIVIDER_H

#include <ros/ros.h>
#include <global_planner/KDTree.h>
#include <global_planner/utils.h>
#include <map_manager/occupancyMap.h>
#include <visualization_msgs/MarkerArray.h>


namespace timeOptimizer{
	class trajDivider{
	private:
		ros::NodeHandle nh_;
		ros::Publisher brakingZoneVisPub_;
		ros::Publisher kdtreeRangeVisPub_;
		ros::Timer visTimer_;


		std::shared_ptr<mapManager::occMap> map_;
		std::vector<Eigen::Vector3d> trajectory_;
		std::vector<double> time_;
		bool complete_ = false;
		int maxLengthIdx_;
		std::vector<bool> mask_;
		std::pair<Eigen::Vector3d, Eigen::Vector3d> sampleRange_;
		std::shared_ptr<KDTree::KDTree<3, int>> kdtree_;
		std::vector<Eigen::Vector3d> nearestObstacles_;
		std::vector<std::pair<double, double>> tInterval_;

		// parameter
		// double maxLength_ = 7.0; // m
		double maxLength_ = 20.0; // m
		double safeDist_ = 1.0; // m
		double minTimeIntervalRatio_ = 0.1;
		double minTime_ = 0.5;
		double minIntervalDiffRatio_ = 0.05;
		double minTimeDiff_ = 0.25;

		

	public:
		trajDivider(const ros::NodeHandle& nh);
		void setMap(const std::shared_ptr<mapManager::occMap>& map);
		void setTrajectory(const std::vector<Eigen::Vector3d>& trajectory, const std::vector<double>& time);
		void registerPub();
		void registerCallback();

		void visCB(const ros::TimerEvent&);

		void run(std::vector<std::pair<double, double>>& tInterval, std::vector<double>& obstacleDist);
		void findRange(Eigen::Vector3d& rangeMin, Eigen::Vector3d& rangeMax);
		void buildKDTree(const Eigen::Vector3d& rangeMin, const Eigen::Vector3d& rangeMax);
		void findNearestObstacles(std::vector<Eigen::Vector3d>& nearestObstacles, std::vector<bool>& mask);
		void divideTrajectory(const std::vector<Eigen::Vector3d>& nearestObstacles, const std::vector<bool>& mask, std::vector<std::pair<double, double>>& tInterval, std::vector<double>& obstacleDist);
		void getNearestObstacles(std::vector<Eigen::Vector3d>& nearestObstacles, std::vector<bool>& mask);
		void getNearestObstacles(std::vector<Eigen::Vector4d>& nearestObstacles);
		void publishBrakingZoneVisMsg();
		void publishKDTreeRangeMsg();
	};
}

#endif