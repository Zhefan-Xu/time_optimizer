/*
*	File: timeOptimizer.h
*	---------------
*   time optimizer header file
*/


#ifndef TIME_OPTIMIZER_H
#define TIME_OPTIMIZER_H

#include <Eigen/Eigen>
#include <mosek.h>

namespace timeOptimizer{
	class timeOptimizer{
	private:
		double vmax_;
		double amax_;
		std::vector<Eigen::Vector3d> posData_;
		std::vector<Eigen::Vector3d> velData_;
		std::vector<Eigen::Vector3d> accData_;
		std::vector<Eigen::Vector3d> timeData_;


	public:
		timeOptimizer();
		void setLimits(double vmax, double amax);
		void loadTrajectory(const std::vector<Eigen::Vector3d>& posData, 
							const std::vector<Eigen::Vector3d>& velData,
							const std::vector<Eigen::Vector3d>& accData,
							const std::vector<Eigen::Vector3d>& timeData);
		void optimize();
	};
}

#endif