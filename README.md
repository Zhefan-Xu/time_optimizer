# Optimal Trajectory Time Allocation Library
This package implements the Robust Optimal Time Allocation (ROTA) framework which is designed to optimize the time progress of the trajectories temporally and served as a post-processing tool to enhance trajectory time efficiency and safety under uncertainties.

**Author**: [Zhefan Xu](https://zhefanxu.com/), Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [global_planner](https://github.com/Zhefan-Xu/tracking_controller/edit/main/README.md), [trajectory_planner](trajectory_planner), and [map_manager](https://github.com/Zhefan-Xu/trajectory_planner). Please use the following commands for installation:
```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/map_manager.git

cd ~/catkin_ws
catkin_make
```


## II. Example DEMO


The related paper can be found on:

**Zhefan Xu and Kenji Shimada, “Quadcopter Trajectory Time Minimization and Robust Collision Avoidance via Optimal Time Allocation”, axriv, 2023.** [\[paper\]](https://arxiv.org/abs/2309.08544) [\[video\]](https://youtu.be/wI8KGcxsyMI?si=1QfDPrm8s6Hfv8vf)


## III. Citation and Reference:
If you find this work useful, please cite the paper:
```
@article{xu2023quadcopter,
  title={Quadcopter Trajectory Time Minimization and Robust Collision Avoidance via Optimal Time Allocation},
  author={Xu, Zhefan and Shimada, Kenji},
  journal={arXiv preprint arXiv:2309.08544},
  year={2023}
}
```



