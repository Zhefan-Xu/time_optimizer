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
							double dt){
		this->posData_ = posData;
		this->velData_ = velData;
		this->accData_ = accData;
		this->dt_ = dt;
	}

	void timeOptimizer::loadTimeInterval(const std::vector<std::pair<double, double>>& timeInterval){
		this->timeInterval_ = timeInterval;
	}

	void timeOptimizer::divideData(){

	}
	
	void timeOptimizer::optimize(){
		this->divideData();


		// create mosek environment
		MSKrescodee r;
		MSKenv_t env = NULL;
		r = MSK_makeenv(&env, NULL);
		
		// create task
		MSKtask_t task = NULL;
		if (r == MSK_RES_OK){
			r = MSK_maketask(env, 0, 0, &task);

			if (r == MSK_RES_OK){
				MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);


			}

		}

	}
}