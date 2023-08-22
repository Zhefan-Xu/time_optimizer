/*
*	File: trajectoryDivider.cpp
*	---------------
*   trajectory divider implementation
*/

#include <time_optimizer/trajectoryDivider.h>

namespace timeOptimizer{
	trajDivider::trajDivider(const ros::NodeHandle& nh) : nh_(nh){
		this->registerPub();
		this->registerCallback();
	}

	void trajDivider::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void trajDivider::setTrajectory(const std::vector<Eigen::Vector3d>& trajectory, const std::vector<double>& time){
		this->trajectory_ = trajectory;
		this->time_ = time;
	}

	void trajDivider::registerPub(){
		this->brakingZoneVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("trajDivider/braking_zone", 10);
		this->kdtreeRangeVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("trajDivider/kdtree_range", 10);
	}

	void trajDivider::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &trajDivider::visCB, this);
	}

	void trajDivider::visCB(const ros::TimerEvent&){
		this->publishBrakingZoneVisMsg();
		this->publishKDTreeRangeMsg();
	}

	void trajDivider::run(std::vector<std::pair<double, double>>& tInterval, std::vector<double>& obstacleDist){
		ros::Time startTime = ros::Time::now();
		this->complete_ = false;
		// 1. find the range based on the trajectory
		Eigen::Vector3d rangeMin, rangeMax;
		this->findRange(rangeMin, rangeMax);
		// cout << "sample range min: " << rangeMin.transpose() << endl;
		// cout << "sample range max: " << rangeMax.transpose() << endl;

		// 2. build KDTree based on range and map
		this->buildKDTree(rangeMin, rangeMax);
		// cout << "KDtree size: " << this->kdtree_->size() << endl;

		// 3. find nearest point for each trajectory sample point
		std::vector<Eigen::Vector3d> nearestObstacles;
		std::vector<bool> mask;
		this->findNearestObstacles(nearestObstacles, mask);

		// 4. divide trajectory
		this->divideTrajectory(nearestObstacles, mask, tInterval, obstacleDist);
		this->complete_ = true;
		ros::Time endTime = ros::Time::now();
		cout << "[trajDivider]: Divider run time: " << (endTime - startTime).toSec() << endl;
	}

	void trajDivider::findRange(Eigen::Vector3d& rangeMin, Eigen::Vector3d& rangeMax){
		if (this->trajectory_.size() <= 2) return;
		double totalLength = 0.0;

		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		rangeMin = this->trajectory_[0];
		rangeMax = this->trajectory_[0];
		this->maxLengthIdx_ = int(this->trajectory_.size());
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

		this->sampleRange_.first = rangeMin;
		this->sampleRange_.second = rangeMax;
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
		if (not this->kdtree_->empty()){
			for (size_t i=0; i<this->trajectory_.size(); ++i){
				if (int(i) >= this->maxLengthIdx_){
					mask[i] = false;
				}
				else{
					if (i != this->trajectory_.size() - 1){
						Eigen::Vector3d p = this->trajectory_[i];
						KDTree::Point<3> point, nn;
						point[0] = p(0);
						point[1] = p(1);
						point[2] = p(2);
						this->kdtree_->nearestNeighbor(point, nn);
						Eigen::Vector3d pNN (nn[0], nn[1], nn[2]);
						Eigen::Vector3d pNext = this->trajectory_[i+1];
						Eigen::Vector3d velDirection = pNext - p;
						Eigen::Vector3d obDirection = pNN - p;
						if ((p - pNN).norm() <= this->safeDist_ and globalPlanner::angleBetweenVectors(velDirection, obDirection) <= globalPlanner::PI_const/2.0){
							mask[i] = true;
							nearestObstacles[i] = pNN;
						}
					}	
				}
			}
		}
		this->mask_ = mask;
		this->nearestObstacles_ = nearestObstacles;
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
			mValPrev = mVal;
		}

		if (not complete){
			interval.second = this->time_.back();
			intervalIdx.second = int(this->time_.size()) - 1;
			tIntervalRaw.push_back(interval);
			tIntervalRawIdx.push_back(intervalIdx);
			complete = true;
		}

		// merge short-time intervals
		double prevEndTime = 0.0;
		std::vector<std::pair<int, int>> tIntervalIdx;
		for (size_t i=0; i<tIntervalRaw.size(); ++i){
			std::pair<double, double> intervalCurr = tIntervalRaw[i];
			double timeThresh = std::min(this->minTimeIntervalRatio_ * this->time_.back(), this->minTime_);
			if (intervalCurr.second - intervalCurr.first > timeThresh){ // time is too short
				double diffTimeThresh = std::min(this->minIntervalDiffRatio_ * this->time_.back(), this->minTimeDiff_);
				if (intervalCurr.first - prevEndTime > diffTimeThresh){
					tInterval.push_back(intervalCurr);
					tIntervalIdx.push_back(tIntervalRawIdx[i]);
				}
				else{
					if (tInterval.size() == 0){ 
						intervalCurr.first = 0.0;
						tIntervalRawIdx[0].first = 0;
						tInterval.push_back(intervalCurr);
						tIntervalIdx.push_back(tIntervalRawIdx[0]);						
					}
					else{
						tInterval.back().second = intervalCurr.second;
						tIntervalIdx.back().second = tIntervalRawIdx[i].second;
					}
				}
				prevEndTime = tInterval.back().second;	
			}
						
		}
		this->tInterval_ = tInterval;

		// std::vector<std::pair<int, int>> tIntervalIdx;
		// for (size_t i=0; i<tIntervalRaw.size(); ++i){
		// 	std::pair<double, double> intervalCurr = tIntervalRaw[i];
		// 	cout << "raw interval: " << intervalCurr.first << " " << intervalCurr.second << endl;
		// 	if (intervalCurr.second - intervalCurr.first > this->minTimeInterval_){ // time is too short
		// 		tInterval.push_back(intervalCurr);
		// 		tIntervalIdx.push_back(tIntervalRawIdx[i]);
		// 	}
		// }
		// this->tInterval_ = tInterval;

		// for (size_t i=0; i<tInterval.size(); ++i){
		// 	std::pair<double, double> intervalCurr = tInterval[i];
		// 	cout << "raw interval: " << intervalCurr.first << " " << intervalCurr.second << endl;			
		// }


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

	void trajDivider::getNearestObstacles(std::vector<Eigen::Vector3d>& nearestObstacles, std::vector<bool>& mask){
		nearestObstacles = this->nearestObstacles_;
		mask = this->mask_;
	}

	void trajDivider::getNearestObstacles(std::vector<Eigen::Vector4d>& nearestObstacles){
		nearestObstacles.clear();
		for (int i=0; i<int(this->mask_.size()); ++i){
			double maskVal = this->mask_[i];
			Eigen::Vector3d nearestObstacle = this->nearestObstacles_[i];
			Eigen::Vector4d ob (maskVal, nearestObstacle(0), nearestObstacle(1), nearestObstacle(2));
			nearestObstacles.push_back(ob);
		}
	}

	void trajDivider::publishBrakingZoneVisMsg(){
		if (this->complete_){
			// raw obstacle trajectory points
			visualization_msgs::MarkerArray obTrajMarkers;
			int countPointNum = 0;
			for (size_t i=0; i<this->trajectory_.size(); ++i){
				bool inInterval = false;
				double t = this->time_[i];
				for (std::pair<double, double> interval : this->tInterval_){
					// cout << "interval range: " << interval.first << " " << interval.second << endl;
					if (t >= interval.first and t <= interval.second){
						// cout << "t: " << t << endl;
						inInterval = true;
						break;
					}
				}

				if (inInterval){
				// if (this->mask_[i]){
					visualization_msgs::Marker point;
					point.header.frame_id = "map";
					point.header.stamp = ros::Time::now();
					point.ns = "obstacle_trajectory_point";
					point.id = countPointNum;
					point.type = visualization_msgs::Marker::SPHERE;
					point.action = visualization_msgs::Marker::ADD;
					point.pose.position.x = this->trajectory_[i](0);
					point.pose.position.y = this->trajectory_[i](1);
					point.pose.position.z = this->trajectory_[i](2);
					point.lifetime = ros::Duration(0.1);
					point.scale.x = 0.2;
					point.scale.y = 0.2;
					point.scale.z = 0.2;
					point.color.a = 1.0;
					point.color.r = 1.0;
					point.color.g = 0.0;
					point.color.b = 0.0;
					++countPointNum;
					obTrajMarkers.markers.push_back(point);					
				}
			}

		


			this->brakingZoneVisPub_.publish(obTrajMarkers);
		}
	}

	void trajDivider::publishKDTreeRangeMsg(){
		if (this->complete_){
			// sample range
			visualization_msgs::MarkerArray obTrajMarkers;
			visualization_msgs::Marker range;
			range.header.frame_id = "map";
			range.header.stamp = ros::Time::now();
			range.ns = "range box";
			range.id = 0;
			range.type = visualization_msgs::Marker::CUBE;
			range.action = visualization_msgs::Marker::ADD;
			range.pose.position.x = (this->sampleRange_.first(0) + this->sampleRange_.second(0))/2.0;
			range.pose.position.y = (this->sampleRange_.first(1) + this->sampleRange_.second(1))/2.0;
			range.pose.position.z = (this->sampleRange_.first(2) + this->sampleRange_.second(2))/2.0;
			// range.lifetime = ros::Duration(0.1);
			range.scale.x = this->sampleRange_.second(0) - this->sampleRange_.first(0);
			range.scale.y = this->sampleRange_.second(1) - this->sampleRange_.first(1);
			range.scale.z = this->sampleRange_.second(2) - this->sampleRange_.first(2);
			range.color.a = 0.4;
			range.color.r = 0.0;
			range.color.g = 0.0;
			range.color.b = 1.0;
			obTrajMarkers.markers.push_back(range);	

			this->kdtreeRangeVisPub_.publish(obTrajMarkers);	
		}
	}
}