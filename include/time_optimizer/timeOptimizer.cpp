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

	void timeOptimizer::divideData(std::vector<std::vector<Eigen::Vector3d>>& posDataList, 
						std::vector<std::vector<Eigen::Vector3d>>& velDataList,
						std::vector<std::vector<Eigen::Vector3d>>& accDataList,
						std::vector<bool>& obstacleInfoList){
		cout << "start divide data based on the time interval." << endl;

		if (this->timeInterval_.size() == 0){
			posDataList.push_back(this->posData_);
			velDataList.push_back(this->velData_);
			accDataList.push_back(this->accData_);
			obstacleInfoList.push_back(false);
		}
		else{
			double t = 0;
			int intervalIdx = 0;
			bool prevInObstacleRange = false;
			bool inObstacleRange = false;
			std::vector<Eigen::Vector3d> posDataDivide, velDataDivide, accDataDivide;
			for (int i=0; i<int(this->posData_.size()); ++i){
				std::pair<double, double> interval = this->timeInterval_[intervalIdx];
				if (t >= interval.first and t <= interval.second){
					inObstacleRange = true;
				}
				else{
					inObstacleRange = false;
				}

				posDataDivide.push_back(this->posData_[i]);
				velDataDivide.push_back(this->velData_[i]);
				accDataDivide.push_back(this->accData_[i]);

				if (i == 0){
					if (inObstacleRange){
						obstacleInfoList.push_back(true);
					}
					else{
						obstacleInfoList.push_back(false);
					}
				}
				else{
					if (prevInObstacleRange and not inObstacleRange){
						posDataList.push_back(posDataDivide);
						velDataList.push_back(velDataDivide);
						accDataList.push_back(accDataDivide);
						obstacleInfoList.push_back(true);

						posDataDivide = std::vector<Eigen::Vector3d> {this->posData_[i]};
						velDataDivide = std::vector<Eigen::Vector3d> {this->velData_[i]};
						accDataDivide = std::vector<Eigen::Vector3d> {this->accData_[i]};
						if (intervalIdx != int(this->timeInterval_.size()-1)){
							++intervalIdx;
						}
					}
					else if (not prevInObstacleRange and inObstacleRange){
						posDataList.push_back(posDataDivide);
						velDataList.push_back(velDataDivide);
						accDataList.push_back(accDataDivide);
						obstacleInfoList.push_back(false);

						posDataDivide = std::vector<Eigen::Vector3d> {this->posData_[i]};
						velDataDivide = std::vector<Eigen::Vector3d> {this->velData_[i]};
						accDataDivide = std::vector<Eigen::Vector3d> {this->accData_[i]};						
					}
				}
				prevInObstacleRange = inObstacleRange;

				t += this->dt_;
			}

			posDataList.push_back(posDataDivide);
			velDataList.push_back(velDataDivide);
			accDataList.push_back(accDataDivide);
			obstacleInfoList.push_back(prevInObstacleRange);
		}

	}
	
	void timeOptimizer::optimize(){
		std::vector<std::vector<Eigen::Vector3d>> posDataList, velDataList, accDataList;
		std::vector<bool> obstacleInfoList;
		this->divideData(posDataList, velDataList, accDataList, obstacleInfoList);


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