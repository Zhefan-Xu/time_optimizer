/*
*	File: bsplineTimeOptimizer.h
*	---------------
*   bspline time optimizer header
*/

#ifndef BSPLINE_TIME_OPTIMIZER_H
#define BSPLINE_TIME_OPTIMIZER_H
#include <memory>
#include <trajectory_planner/bspline.h>
#include <time_optimizer/trajectoryDivider.h>
#include <time_optimizer/timeOptimizer.h>

namespace timeOptimizer{
	class bsplineTimeOptimizer{
	private:
		ros::NodeHandle nh_;
		trajPlanner::bspline traj_;
		double linearReparamFactor_;
		std::shared_ptr<trajDivider> trajDivider_;
		std::shared_ptr<timeOptimizer> timeOpt_;
		bool optimizeSuccess_ = false;

	public:
		bsplineTimeOptimizer(const ros::NodeHandle& nh);
		void setMap(const std::shared_ptr<mapManager::occMap>& map);
		void optimize(trajPlanner::bspline traj, double vmax, double amax, double dt);
		double getStates(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& acc);
		double getLinearReparamFactor(trajPlanner::bspline traj, double vmax, double amax);
		double getDuration();
	};
}

#endif