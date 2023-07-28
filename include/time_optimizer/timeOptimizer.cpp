/*
*	File: timeOptimizer.h
*	---------------
*   time optimizer implementation
*/

#include <time_optimizer/timeOptimizer.h>

namespace timeOptimizer{
	timeOptimizer::timeOptimizer(){}
	

	void timeOptimizer::setLimits(double vmax, double amax){
		this->vmax_ = vmax;
		this->amax_ = amax;
	}

	void timeOptimizer::loadTrajectory(const std::vector<Eigen::Vector3d>& posData, 
							const std::vector<Eigen::Vector3d>& velData,
							const std::vector<Eigen::Vector3d>& accData,
							const std::vector<Eigen::Vector3d>& timeData){
		this->posData_ = posData;
		this->velData_ = velData;
		this->accData_ = accData;
		this->timeData_ = timeData;
	}
	
	void timeOptimizer::optimize(){
		// create mosek environment
		MSKrescodee r;
		MSKenv_t env = NULL;
		r = MSK_makeenv(&env, NULL);
		int i = 0;

	}
}