/*
*	File: timeOptimizer.h
*	---------------
*   time optimizer header file
*/


#ifndef TIME_OPTIMIZER_H
#define TIME_OPTIMIZER_H

#include <iostream>
#include <Eigen/Eigen>
#include <time_optimizer/third_party/include/mosek.h>
#include <time_optimizer/utils.h>
using std::cout; using std::endl;

// static void MSKAPI printstr(void *handle,
//                             const char str[]){
//   	printf("%s", str);
// } 

namespace timeOptimizer{
	class timeOptimizer{
	private:
		double alphaCollision_ = 0.01;
		double obstacleStd_ = 0.1;
		double lambda_ = 0.5;
		double vmax_;
		double vob_; // velocity for meeting obstacles
		double amax_;
		double dt_;
		std::vector<Eigen::Vector3d> posData_;
		std::vector<Eigen::Vector3d> velData_;
		std::vector<Eigen::Vector3d> accData_;
		std::vector<Eigen::Vector4d> obstacleData_;
		std::vector<double> velocityLimits_;
		std::vector<std::pair<double, double>> timeInterval_;

		std::vector<double> trajTime_;
		std::vector<double> realTime_;
		std::vector<double> alphaSol_;
		std::vector<double> betaSol_;

	public:
		timeOptimizer();
		void setLimits(double vmax, double amax);
		void loadTrajectory(const std::vector<Eigen::Vector3d>& posData, 
							const std::vector<Eigen::Vector3d>& velData,
							const std::vector<Eigen::Vector3d>& accData,
							double dt);
		void loadObstacles(const std::vector<Eigen::Vector4d>& obstacleData);
		void loadTimeInterval(const std::vector<std::pair<double, double>>& timeInterval);
		void divideData(std::vector<std::vector<Eigen::Vector3d>>& posDataList, 
						std::vector<std::vector<Eigen::Vector3d>>& velDataList,
						std::vector<std::vector<Eigen::Vector3d>>& accDataList,
						std::vector<bool>& obstacleInfoList,
						std::vector<std::vector<double>>& velocityLimitsList);
		bool optimize();
		void extractSol(const std::vector<double>& sol);
		double remapTime(double tau, double& alpha, double& beta);
		void getVelocityLimits();
		double cov2vel(double var);
		double getDuration();
		double getMaxVel();
		double getMaxAcc();
	};
}

#endif