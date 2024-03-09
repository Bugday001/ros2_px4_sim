#ifndef _SIMPLE_TRAJ_H
#define _SIMPLE_TRAJ_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
using namespace std;
class SimpleTraj
{
private:
    std::vector<Eigen::Vector3d> way_points_;
    Eigen::Vector3d euler_;
    double max_vel_, max_acc_;
    double speedUpLen_, totalLen_;
    double speedUpTime_, totalTime_;
public:
    SimpleTraj();
    void setParams(double max_vel, double max_acc);
    void setWayPoints(std::vector<Eigen::Vector3d>& way_points);
    bool getPVA(vector<Eigen::Vector3d>& pva, double t);
};

SimpleTraj::SimpleTraj()
{
    max_vel_ = 0.1;
    max_acc_ = 0.1;
}

void SimpleTraj::setParams(double max_vel, double max_acc) 
{
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    speedUpTime_ = max_vel_/max_acc_;
}

void SimpleTraj::setWayPoints(std::vector<Eigen::Vector3d>& way_points)
{
    way_points_ = way_points;
    Eigen::Vector3d delta_xyz = way_points[1] - way_points[0];
    totalLen_ = delta_xyz.norm()+1e-14;
    euler_[0] = delta_xyz[0]/totalLen_;
    euler_[1] = delta_xyz[1]/totalLen_;
    euler_[2] = delta_xyz[2]/totalLen_;
    speedUpLen_ = 0.5*max_acc_*speedUpTime_*speedUpTime_;
    if(2*speedUpLen_ > totalLen_) {
        totalTime_ = 2 * sqrt(totalLen_/max_acc_);
    }
    else {
        totalTime_ = (totalLen_-speedUpLen_*2)/max_vel_ + 2*speedUpTime_;
    }
}

/**
 * 返回是否完成
*/
bool SimpleTraj::getPVA(vector<Eigen::Vector3d>& pva, double t)
{   
    if(t>totalTime_) {
        return true;
    }
    if(2*speedUpLen_ > totalLen_) {  //不能加到最大速度
        double speedupTime = totalTime_ / 2;
        if(t < speedupTime) {
            pva[0] = way_points_[0] + 0.5*max_acc_*euler_*t*t;
            pva[1] = max_acc_*euler_*t;
            pva[2] = max_acc_*euler_;
        }
        else {
            pva[0] = way_points_[0] + (totalLen_ - 0.5*max_acc_*(totalTime_-t)*(totalTime_-t))*euler_;
            pva[1] = max_acc_*(totalTime_-t)*euler_;
            pva[2] = -max_acc_*euler_;
        }
    }
    else {
        if(t < speedUpTime_) {
            pva[0] = way_points_[0] + 0.5*max_acc_*euler_*t*t;
            pva[1] = max_acc_*euler_*t;
            pva[2] = max_acc_*euler_;
        }
        else if(t<=totalTime_-speedUpTime_) {
            pva[0] = way_points_[0] + (speedUpLen_ + (t-speedUpTime_)*max_vel_)*euler_;
            pva[1] = max_vel_*euler_;
            pva[2].setZero();
        }
        else {
            pva[0] = way_points_[0] + (totalLen_ - 0.5*max_acc_*(totalTime_-t)*(totalTime_-t))*euler_;
            pva[1] = max_acc_*(totalTime_-t)*euler_;
            pva[2] = -max_acc_*euler_;
        }
    }
    return false;
}
#endif