/*
*	File: timeOptimizer.h
*	---------------
*   time optimizer header file
*/


#ifndef TIME_OPTIMIZER_H
#define TIME_OPTIMIZER_H

#include <iostream>
#include <Eigen/Eigen>
#include <mosek.h>
using std::cout; using std::endl;

static void MSKAPI printstr(void *handle,
                            const char str[]){
  	printf("%s", str);
} 

namespace timeOptimizer{
	class timeOptimizer{
	private:
		double vmax_;
		double amax_;
		double dt_;
		std::vector<Eigen::Vector3d> posData_;
		std::vector<Eigen::Vector3d> velData_;
		std::vector<Eigen::Vector3d> accData_;
		std::vector<std::pair<double, double>> timeInterval_;


	public:
		timeOptimizer();
		void setLimits(double vmax, double amax);
		void loadTrajectory(const std::vector<Eigen::Vector3d>& posData, 
							const std::vector<Eigen::Vector3d>& velData,
							const std::vector<Eigen::Vector3d>& accData,
							double dt);
		void loadTimeInterval(const std::vector<std::pair<double, double>>& timeInterval);
		void divideData();
		void optimize();
	};
}

#endif