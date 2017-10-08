#include "traffic.h"
#include <vector>
#include "jmt.h"

/*
 * classify the vehicle into car in front of the ego vehicle,
 * behind the car, or car on the left/right lane etc
 */
void
Traffic::updateNeighboringVehicle(double vid, double x, double y,
    double vx, double vy, double s, double d)
{
    //if (d < 0)
        //return;

    if (vid < 0)
    {
	//skip vehicle with negative vid
	std::cout << "invalid vehicle id detected" << std::endl;
        return;
    }

    int lane = Traffic::getLane(d);
    double distance = Traffic::distance(car_x_, car_y_, x, y);
    double diff = Traffic::calculateSDiff(s, car_s_);
 
    if (lane == car_lane_)
    {
	//in the same lane, look for car in front of the cara
        if (diff > 0.0 && diff < currentlanef_.diff) {
            currentlanef_.update(vid, x, y, vx, vy, s, d, distance, diff);
	} else if (diff < 0.0 && -diff < currentlaneb_.diff) {
	    currentlaneb_.update(vid, x, y, vx, vy, s, d, distance, -diff);
	}
    } else if (lane == leftlaneb_.lane) { 
	//on the left lane, look for car immediately behind the car
        if (diff > 0.0 && diff < leftlanef_.diff)
        {
            //on the left lane, look for car in front of the car
            leftlanef_.update(vid, x, y, vx, vy, s, d, distance, diff);
        } else if (diff < 0.0 && -diff < leftlaneb_.diff) {
           //on the left lane, look for car behind the car
           leftlaneb_.update(vid, x, y, vx, vy, s, d , distance, -diff);
        }
    } else if (lane == rightlaneb_.lane) {
        if (diff > 0.0 && diff < rightlanef_.diff) {
            //on the right lane, look for car in front of the car
	    rightlanef_.update(vid, x, y, vx, vy, s, d, distance, diff);
        } else if (diff < 0.0 && -diff < rightlaneb_.diff) {  
	    //on the right lane, look for car immediately behind the car
            rightlaneb_.update(vid, x, y, vx, vy, s, d, distance, -diff);
        }
    }
}

/*
 * calculates the speed of the car, if the car doesn't exist, then
 * return max target speed
 */
double
Traffic::calculateSpeed(const Car& car) const
{
    return (car.vid > 0)
		?sqrt(car.vx*car.vx + car.vy*car.vy):MAX_TARGET_SPEED;
}

/*
 * get the target lane, and speed
 */
void
Traffic::getTarget(double& target_v, double& target_d)
{
    //find the cost under each scenario, and find the minimum cost scenario
    if (changingLane_)
    {
	target_v = target_v_;
	target_d = target_d_;
/*
        double frontspeed = calculateSpeed(currentlanef_);
        double backspeed  = calculateSpeed(currentlaneb_);
        double frontdiff  = (currentlanef_.vid > 0)?currentlanef_.diff:(std::numeric_limits<double>::max());
        double backdiff   = (currentlaneb_.vid > 0)?currentlaneb_.diff:(std::numeric_limits<double>::max());
        if (target_d == leftLane())
        {
            frontspeed = calculateSpeed(leftlanef_);
            backspeed  = calculateSpeed(leftlaneb_);
            frontdiff = (leftlanef_.vid > 0)?leftlanef_.diff:(std::numeric_limits<double>::max());
            backdiff = (leftlaneb_.vid > 0)?leftlaneb_.diff:(std::numeric_limits<double>::max());
        } else if (target_d == rightLane()) {
            frontspeed = calculateSpeed(rightlanef_);
            backspeed = calculateSpeed(rightlaneb_);
            frontdiff = (rightlanef_.vid > 0)?rightlanef_.diff:(std::numeric_limits<double>::max());
            backdiff = (rightlaneb_.vid > 0)?rightlaneb_.diff:(std::numeric_limits<double>::max());
        }

        //do i need to slow slow
        if (frontdiff < SAFE_DISTANCE) {
            //car in front is too close, slow down
            target_v = std::min(target_v, 0.9*frontspeed);
        } else if (backdiff < SAFE_DISTANCE && target_v < backspeed) {
            //too slow, speed up
            target_v = std::max(target_v, backspeed);
        }
*/

	if (std::abs(target_d_ - car_d_) < 0.1)
	{
	    changingLane_ = false;
	    inlanecount_ = 10;
	}
	return;
    }

    double laneSpeed = calculateSpeed(currentlanef_);
    double minCost = sameLaneCost(laneSpeed);
    int lane = car_lane_;
    double s_speed = Traffic::getMaxSpeed(lane);
    laneSpeed = calculateSpeed(leftlanef_);
    double cost = calculateLaneCost(laneSpeed, leftLane(),
			leftlanef_, leftlaneb_);
    if (inlanecount_ == 0 && cost < minCost)
    {
	lane = leftLane();
	s_speed = (leftlanef_.vid >= 0 && leftlanef_.diff < SAFE_DISTANCE)
			?0.9*laneSpeed:Traffic::getMaxSpeed(lane);
	minCost = cost;
    }
    laneSpeed = calculateSpeed(rightlanef_);
    cost = calculateLaneCost(laneSpeed, rightLane(), rightlanef_,
			rightlaneb_);
    if (inlanecount_ == 0 && cost < minCost)
    {
	lane = rightLane();
	s_speed = (rightlanef_.vid >= 0 && rightlanef_.diff < SAFE_DISTANCE)
			?0.9*laneSpeed:Traffic::getMaxSpeed(lane);
	minCost = cost;
    }
    laneSpeed = calculateSpeed(currentlanef_);
    cost = slowdownCost(laneSpeed);
    if (cost < minCost)
    {
	lane = car_lane_;
	if (currentlanef_.vid > 0)
        {
	    s_speed = 0.9*laneSpeed; 
	    //set speed to be slightly less than the leading car
	    minCost = cost;
        }
    }
    if (inlanecount_ > 0)
	--inlanecount_;
    if (lane != car_lane_)
    {
	changingLane_ = true;
    }
    target_d = 4*lane - 2;
    //if (lane == 3)
	//target_d -= 0.01; //avoid going outside the lane
    target_v = std::min(s_speed, Traffic::getMaxSpeed(lane));
}
 
double
Traffic::sameLaneCost(double speed)
{
    double cost = (currentlanef_.vid < 0 || currentlanef_.diff > SAFE_DISTANCE)
			?0:COLLISION_COST;
    double maxspeed = Traffic::getMaxSpeed(car_lane_);
    cost += 30*std::abs(maxspeed - 
			std::max(speed, maxspeed))/maxspeed;
    cost += Traffic::lanecost(car_lane_);
    return cost;
}

double
Traffic::calculateLaneCost(double speed, int lane, const Car& frontcar, const Car& backcar)
{
    double cost = COLLISION_COST;
    double final_s = car_s_ + car_speed_*TIME_STEP*WINDOW_SIZE;
    double backspeed = (backcar.vid > 0)?calculateSpeed(backcar):0;
    double maxspeed = Traffic::getMaxSpeed(lane);
    double frontspeed = (frontcar.vid > 0)?calculateSpeed(frontcar):2*maxspeed;
    if (lane < 0)
    {
	cost = COLLISION_COST;
    } else if (frontcar.vid > 0 && calculateSDiff(final_s, frontcar.s) > 0) {
	cost = COLLISION_COST;
    } else if ((frontcar.vid < 0 || frontcar.diff > safeDistanceAhead(car_speed_, frontspeed)) 
		&& (backcar.vid < 0 || backcar.diff > safeDistanceFromBehind(car_speed_, backspeed))) {
	cost = TURN_PREFER_COST;
    } else if ((frontcar.vid < 0 || frontcar.diff > SAFE_DISTANCE_FRONT) &&
	(backcar.vid < 0 || backcar.diff > SAFE_DISTANCE_FROM_BEHIND)) {
	//safe to switch to left lane, has to keep safe distance from
        //both front and car behind in left lane
	cost = TURN_COST;	
    } else {
	//not safe to switch to left lane
	cost = COLLISION_COST;
    }
    maxspeed = Traffic::getMaxSpeed(lane);
    cost += 30*std::abs(maxspeed - 
			std::max(speed, maxspeed))/maxspeed;
    cost += Traffic::lanecost(lane);
    return cost;
}

double
Traffic::slowdownCost(double speed)
{
    double cost = SLOWDOWN_COST;
    double maxspeed = Traffic::getMaxSpeed(car_lane_);
    cost += 30*std::abs(maxspeed - speed)/maxspeed;
    cost += lanecost(car_lane_);
    return cost;
}

void
Traffic::getTrajectory(const std::vector<double>& prev_x, 
		const std::vector<double>& prev_y,
		std::vector<double>& nexts, std::vector<double>& nextd)
{
    int prev_size = prev_x.size();
    int num_to_keep = std::min((int)s_window_.size(), prev_size);
    int num_to_remove = s_window_.size() - num_to_keep;
    if (num_to_remove > 0)
    {
	s_window_.erase(s_window_.begin(), s_window_.begin() + num_to_remove);
	d_window_.erase(d_window_.begin(), d_window_.begin() + num_to_remove);
    }
    //s and d are vector of 3 elements, s, s_dot and s_dot_dot,
    //d, d_dot and d_dot_dot, to pass into jmt method
    std::vector<double> s(3);
    std::vector<double> d(3);
    double car_speed = 0;
    if (prev_size < 2)
    {
	s[0] = car_s_;
        d[0] = car_d_;
	s[1] = s[2] = 0;
	d[1] = d[2] = 0;
    } else {
	int last = s_window_.size() - 1;
	s[0] = s_window_[last];
	d[0] = d_window_[last];
	s[1] = velocity(s_window_, last);
	s[2] = acceleration(s_window_, last);
        d[1] = velocity(d_window_, last);
	d[2] = acceleration(d_window_, last);
        car_speed = s[1];	
    }
    int points_to_add = WINDOW_SIZE - s_window_.size();
    double elapsedTime = TIME_STEP * points_to_add;
    //populate final s and d value
    prev_target_v_ = car_speed;
    if (target_v_ > prev_target_v_)
        target_v_ = std::min(prev_target_v_ + 0.25*elapsedTime*MAX_ACCELERATION, target_v_);
    else //decelerate
	target_v_ = std::max(prev_target_v_ - 0.5*elapsedTime*MAX_ACCELERATION, target_v_);

    //setup parameter to calculate trajectory which minimize jerk, acceleration
    int lane = Traffic::getLane(target_d_);
    double maxspeed = Traffic::getMaxSpeed(lane);
    target_v_ = std::min(target_v_, maxspeed);
    prev_target_v_ = target_v_;
    double final_s  = s[0] + 0.5*(target_v_ + prev_target_v_)*elapsedTime; //target s position after elapsedTime
    std::vector<double> finalS = { final_s, target_v_, 0.0 };
    std::vector<double> finalD = { target_d_, 0.0, 0.0};
    std::vector<double> s_coeffs = getMinJerk(s, finalS, elapsedTime);
    //to get smooth transition in d direction, has to use longer time period
    std::vector<double> d_coeffs = getMinJerk(d, finalD, TIME_STEP*D_WINDOW_SIZE);

    debug();

    for (int i = 0; i < points_to_add; ++i)
    {
	double t = (i+1)*TIME_STEP;
	double pos_s = std::fmod(evaluate(s_coeffs, t), MAX_S);
	double pos_d = (car_d_ == target_d_)?target_d_:evaluate(d_coeffs, t);

	s_window_.push_back(pos_s);
	d_window_.push_back(pos_d);

	nexts.push_back(pos_s);
	nextd.push_back(pos_d);
    }
}

void
Traffic::debug()
{
    std::cout << "current lane " << car_lane_ << ", s(" << car_s_ << "," << car_d_ << "), xy (" << car_x_ << "," << car_y_ << "), speed " << car_speed_ << std::endl;
    std::cout << "front (" << currentlanef_.vid << "," << currentlanef_.s << "," << currentlanef_.d << "), diff " << currentlanef_.diff << ", speed " << calculateSpeed(currentlanef_) << std::endl;
    std::cout << "behind (" << currentlaneb_.vid << "," << currentlaneb_.s << "," << currentlaneb_.d << "), diff " << currentlaneb_.diff << ", speed " << calculateSpeed(currentlaneb_) << std::endl;
    std::cout << "changing lane: " << changingLane_ << ", inlane count " << inlanecount_ << ", target d:" << target_d_ << ", target v: " << target_v_ << std::endl; 
    std::cout << "left lane " << leftLane() << ", left front : (" << leftlanef_.vid << "," << leftlanef_.s << "," << leftlanef_.d << "), diff " << leftlanef_.diff << ", speed " << calculateSpeed(leftlanef_) << std::endl;
    std::cout << "left lane behind: (" << leftlaneb_.vid << "," << leftlaneb_.s << "," << leftlaneb_.d << "), diff " << leftlaneb_.diff << ", speed: " << calculateSpeed(leftlaneb_) << std::endl;
    std::cout << "right lane " << rightLane() << ", right front : (" << rightlanef_.vid << "," << rightlanef_.s << "," << rightlanef_.d << "), diff " << rightlanef_.diff << ", speed " << calculateSpeed(rightlanef_) << std::endl;
    std::cout << "right lane behind: (" << rightlaneb_.vid << "," << rightlaneb_.s << "," << rightlaneb_.d << "), diff " << rightlaneb_.diff << ", speed " << calculateSpeed(rightlaneb_) << std::endl;
    std::cout << "*********************" << std::endl;
}

double
Traffic::velocity(const std::vector<double>& window, int last)
{
    double result = 0.0;
    if (last < 1 || last >= window.size())
    {
	std::cerr << "invalid index " << last << ", while window has size "
		    << window.size() << std::endl;
    } else {
	double diff = Traffic::calculateSDiff(window[last], window[last-1]);
	result = diff/TIME_STEP;
    }
    return result;
}

double
Traffic::acceleration(const std::vector<double>& window, int last)
{
    double result = 0.0;
    if (last < 2 || last >= window.size())
    {
	std::cerr << "invalid index " << last << ", while window has size "
	            << window.size() << std::endl;
    } else {
	double v1 = velocity(window, last);
	double v2 = velocity(window, last-1);

	result = (v1 - v2)/TIME_STEP;
    }
    return result;
}

