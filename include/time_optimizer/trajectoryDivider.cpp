/*
*	File: dep.cpp
*	---------------
*   dynamic exploration planner implementation
*/

#include <time_optimizer/trajectoryDivider.h>

namespace timeOptimizer{
	trajDivider::trajDivider(const ros::NodeHandle& nh) : nh_(nh){
		
	}

	void trajDivider::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void trajDivider::setTrajectory(const std::vector<Eigen::Vector3d>& trajectory, const std::vector<double>& time){
		this->trajectory_ = trajectory;
		this->time_ = time;
	}

	void trajDivider::run(std::vector<std::pair<double, double>>& tInterval, std::vector<double>& obstacleDist){
		// 1. find the range based on the trajectory
		Eigen::Vector3d rangeMin, rangeMax;
		this->findRange(rangeMin, rangeMax);
		cout << "sample range min: " << rangeMin.transpose() << endl;
		cout << "sample range max: " << rangeMax.transpose() << endl;

		// 2. build KDTree based on range and map
		this->buildKDTree(rangeMin, rangeMax);
		cout << "KDtree size: " << this->kdtree_->size() << endl;

		// 3. find nearest point for each trajectory sample point
		std::vector<Eigen::Vector3d> nearestObstacles;
		std::vector<bool> mask;
		this->findNearestObstacles(nearestObstacles, mask);

		// 4. divide trajectory
		this->divideTrajectory(nearestObstacles, mask, tInterval, obstacleDist);
	}

	void trajDivider::findRange(Eigen::Vector3d& rangeMin, Eigen::Vector3d& rangeMax){
		if (this->trajectory_.size() <= 2) return;
		double totalLength = 0.0;

		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		rangeMin = this->trajectory_[0];
		rangeMax = this->trajectory_[0];
		for (size_t i=1; i<this->trajectory_.size(); ++i){
			Eigen::Vector3d pPrev = this->trajectory_[i-1];
			Eigen::Vector3d pCurr = this->trajectory_[i];
			totalLength += (pCurr - pPrev).norm();
			if (totalLength > this->maxLength_){
				this->maxLengthIdx_ = i;
				break;
			}

			// x update
			if (pCurr(0) < rangeMin(0)){
				rangeMin(0) = pCurr(0);
			}
			else if (pCurr(0) > rangeMax(0)){
				rangeMax(0) = pCurr(0);
			}

			// y update
			if (pCurr(1) < rangeMin(1)){
				rangeMin(1) = pCurr(1);
			}
			else if (pCurr(1) > rangeMax(1)){
				rangeMax(1) = pCurr(1);
			}

			// z update
			if (pCurr(2) < rangeMin(2)){
				rangeMin(2) = pCurr(2);
			}
			else if (pCurr(2) > rangeMax(2)){
				rangeMax(2) = pCurr(2);
			}
		}

		// inflate this region
		Eigen::Vector3d inflateVec (this->safeDist_, this->safeDist_, 0.0);
		rangeMin -= inflateVec;
		rangeMax += inflateVec;

		// compare with the map range
		if (rangeMin(0) < mapMin(0)){
			rangeMin(0) = mapMin(0);
		}

		if (rangeMin(1) < mapMin(1)){
			rangeMin(1) = mapMin(1);
		}

		if (rangeMin(2) < mapMin(2)){
			rangeMin(2) = mapMin(2);
		}
		
		if (rangeMax(0) > mapMax(0)){
			rangeMax(0) = mapMax(0);
		}

		if (rangeMax(1) > mapMax(1)){
			rangeMax(1) = mapMax(1);
		}

		if (rangeMax(2) > mapMax(2)){
			rangeMax(2) = mapMax(2);
		}
	}

	void trajDivider::buildKDTree(const Eigen::Vector3d& rangeMin, const Eigen::Vector3d& rangeMax){
		// check all voxels in the sample range and add them to KDTree
		std::vector<std::pair<KDTree::Point<3>, int>> points;
		KDTree::Point<3> point;
		double res = this->map_->getRes();
		for (double x=rangeMin(0); x<=rangeMax(0); x+=res){
			for (double y=rangeMin(1); y<=rangeMax(1); y+=res){
				for (double z=rangeMin(2); z<=rangeMax(2); z+=res){
					Eigen::Vector3d p (x, y, z);
					bool occupied = this->map_->isInflatedOccupied(p);
					if (occupied){
						point[0] = p(0);
						point[1] = p(1);
						point[2] = p(2);
						points.push_back(std::make_pair(point, 0));
					}
				}
			}
		}

		this->kdtree_.reset(new KDTree::KDTree<3, int> (points));
	}

	void trajDivider::findNearestObstacles(std::vector<Eigen::Vector3d>& nearestObstacles, std::vector<bool>& mask){
		nearestObstacles.resize(this->trajectory_.size());
		mask.resize(this->trajectory_.size(), false);
		for (size_t i=0; i<this->trajectory_.size(); ++i){
			if (int(i) >= this->maxLengthIdx_){
				mask[i] = false;
			}
			else{
				Eigen::Vector3d p = this->trajectory_[i];
				KDTree::Point<3> point, nn;
				point[0] = p(0);
				point[1] = p(1);
				point[2] = p(2);
				this->kdtree_->nearestNeighbor(point, nn);
				Eigen::Vector3d pNN (nn[0], nn[1], nn[2]);
				if ((p - pNN).norm() <= this->safeDist_){
					mask[i] = true;
					nearestObstacles[i] = pNN;
				}	
			}
		}
	}

	void trajDivider::divideTrajectory(const std::vector<Eigen::Vector3d>& nearestObstacles, const std::vector<bool>& mask, std::vector<std::pair<double, double>>& tInterval, std::vector<double>& obstacleDist){
		// find the raw interval
		std::vector<std::pair<double, double>> tIntervalRaw;
		std::vector<std::pair<int, int>> tIntervalRawIdx;
		std::pair<double, double> interval;
		std::pair<int, int> intervalIdx;
		bool mValPrev = false;
		bool complete = true;
		for (size_t i=0; i<mask.size(); ++i){
			bool mVal = mask[i];
			if (mValPrev == false and mVal == true){
				interval.first = this->time_[i];
				intervalIdx.first = i;
				complete = false;
			}
			else if (mValPrev == true and mVal == false){
				interval.second = this->time_[i-1];
				intervalIdx.second = i-1;
				tIntervalRaw.push_back(interval);
				tIntervalRawIdx.push_back(intervalIdx);
				complete = true;
			}
		}

		if (not complete){
			interval.second = this->time_.back();
			intervalIdx.second = int(this->time_.size()) - 1;
			tIntervalRaw.push_back(interval);
			tIntervalRawIdx.push_back(intervalIdx);
			complete = true;
		}

		// merge short-time intervals
		std::vector<std::pair<int, int>> tIntervalIdx;
		for (size_t i=0; i<tIntervalRaw.size(); ++i){
			std::pair<double, double> intervalCurr = tIntervalRaw[i];
			if (intervalCurr.second - intervalCurr.first > this->minTimeInterval_){ // time is too short
				tInterval.push_back(intervalCurr);
				tIntervalIdx.push_back(tIntervalRawIdx[i]);
			}				
		}

		// compute the minimum distance for each time interval
		for (size_t i=0; i<tInterval.size(); ++i){
			double minDist = std::numeric_limits<double>::max();
			for (int j=tIntervalIdx[i].first; j<=tIntervalIdx[i].second; ++j){
				double dist = (nearestObstacles[i] - this->trajectory_[i]).norm();
				if (dist < minDist){
					minDist = dist;
				}
			}
			obstacleDist.push_back(minDist);
		}
		
	}

}