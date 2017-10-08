#ifndef _TRAFFIC_H_
#define _TRAFFIC_H_

#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include "const.h"

struct Car {
    //vid -1 implies invalid. e.g no car in front of the car
    Car() : vid(-1), x(0), y(0), vx(0), vy(0), s(0), d(0), lane(0), distance(std::numeric_limits<double>::max()), diff(std::numeric_limits<double>::max()) {}

    void update(double in_vid, double in_x, double in_y, double in_vx, 
                double in_vy, double in_s, double in_d, double in_distance,
                double in_diff)
    {
	vid      = in_vid;
	x        = in_x;
	y        = in_y;
	vx       = in_vx;
	vy       = in_vy;
	s        = in_s;
	d        = in_d;
	distance = in_distance;
        diff     = in_diff;
    }

    void reset(int in_lane) 
    {
	lane = in_lane;
	vid = -1;
	diff = 2*SAFE_DISTANCE;
        distance = std::numeric_limits<double>::max();
    }

    double vid;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    int    lane;
    double diff;
    double distance;
};

class Traffic {
public:
    Traffic() : car_lane_(-1), car_x_(0), car_y_(0), car_s_(0),
                car_d_(0), car_yaw_(-1), car_speed_(0), 
                target_v_(0), target_d_(6), changingLane_(false),
		inlanecount_(0) {}
   ~Traffic() {}

    static int getLane(double d) {
	int lane = 1;
        if (d <= 4.0)
	   lane = 1;
	else if (d <= 8.0)
	   lane = 2;
	else
	   lane = 3;
	return lane; 
    }
    static double getMaxSpeed(int lane)
    {
	if (lane == 3)
	    return MAX_TARGET_SPEED*0.96;
	else if (lane == 2)
	    return MAX_TARGET_SPEED*0.98;
	return MAX_TARGET_SPEED; 
    }
    static int lanecost(int lane)
    {
	int cost = 10;
	if (lane == 2)
	    cost = 0;
	else if (lane == 1)
	    cost = 5;
	return cost;
    }
    static double safeDistanceFromBehind(double carspeed, double backspeed)
    {
	double d = SAFE_DISTANCE_FROM_BEHIND;
        if (carspeed < backspeed && backspeed - carspeed > 5)
	    d = 2*SAFE_DISTANCE_FROM_BEHIND;
	return d;
    }
    static double safeDistanceAhead(double carspeed, double frontspeed)
    {
	double d = SAFE_DISTANCE_FRONT;
	if (carspeed > frontspeed && carspeed - frontspeed > 5)
	    d = 2*SAFE_DISTANCE_FRONT;
	return d; 
    }
    static double calculateSDiff(double s1, double s2)
    {
	//track is circular
	double diff = s1 - s2;
	if (diff > 0.5*MAX_S)
	    diff -= MAX_S;
	else if (diff < -0.5*MAX_S)
	    diff += MAX_S;
	return diff;
    }
    static double distance(double x1, double y1, double x2, double y2)
    {
	return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    }
    static constexpr double pi() { return M_PI; }
    static double deg2rad(double x) { return x*pi()/180.0; }
    static double rad2deg(double x) { return x*180/pi(); }

    template <typename T> 
    void update(double x, double y, double s, double d, double speed, 
                double yaw, const T& sensor_fusion);
    void getTrajectory(const std::vector<double>& prev_x, 
		const std::vector<double>& prev_y,
		std::vector<double>& nextx, std::vector<double>& nexty);
    int  leftLane()  const { return ((car_lane_+1) > 3)?-1:(car_lane_+1);}
    int  rightLane() const { return ((car_lane_-1) > 0)?(car_lane_-1):-1;}

    double getTargetV() const { return target_v_;}
    double getTargetD() const { return target_d_; } 
    static constexpr double COLLISION_COST = 500;
    static constexpr double SLOWDOWN_COST = 200;
    static constexpr double TURN_COST = 150;
    static constexpr double TURN_PREFER_COST = 100;
private:
    void getTarget(double& target_v, double& target_d);
    void updateNeighboringVehicle(double vid, double x, double y,
                double vx, double vy, double s, double d);

    double sameLaneCost(double speed);
    double calculateLaneCost(double speed, int lane, const Car& frontcar,
		const Car& backcar);
    double slowdownCost(double speed);
  
    double velocity(const std::vector<double>& window, int index);
    double acceleration(const std::vector<double>& window, int index);
    double calculateSpeed(const Car& car) const;

    void debug();

    //if there is no car in front, or immediately after our car on
    //left or right lane, the corresponding vid will be -1
    Car currentlanef_;  //car in front of the car
    Car currentlaneb_;  //car behind the car
    Car leftlaneb_;     //car on the left lane immediately after our car
    Car leftlanef_;     //car on the left lane in front of our car
    Car rightlaneb_;    //car on the right lane immediately after our car
    Car rightlanef_;    //car on the right lane in front of our car

    int    car_lane_; //lane # starts from 1
    double car_x_;
    double car_y_;
    double car_s_;
    double car_d_;
    double car_yaw_;
    double car_speed_;

    double target_v_;
    double prev_target_v_;
    double target_d_;
    bool   changingLane_;
    int    inlanecount_; //if inlanecount_ > 0, stay in the same lane
                         //to avoid excessive lane changing

    std::vector<double> s_window_;  //keep track of s values
    std::vector<double> d_window_;  //keep track of d values
};

template <typename T>
void Traffic::update(double x, double y, double s, double d, 
                  double speed, double yaw, const T& sensor_fusion)
{
    car_x_            = x;
    car_y_            = y;
    car_s_            = s;
    car_d_            = d;
    car_yaw_          = yaw;
    car_speed_        = speed;
    car_lane_         = getLane(d);
    currentlanef_.reset(car_lane_);
    currentlaneb_.reset(car_lane_);
    leftlaneb_.reset(leftLane());
    leftlanef_.reset(leftLane());
    rightlaneb_.reset(rightLane());
    rightlanef_.reset(rightLane());

    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
	auto vehicle = sensor_fusion[i];
	double vid = vehicle[0];
        double v_x = vehicle[1];
        double v_y = vehicle[2];
        double v_vx = vehicle[3];
        double v_vy = vehicle[4];
        double v_s  = vehicle[5];
        double v_d  = vehicle[6];
 
        updateNeighboringVehicle(vid, v_x, v_y, v_vx, v_vy, v_s, v_d); 
    }
    getTarget(target_v_, target_d_);
}

#endif //traffic.h
