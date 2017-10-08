# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
. The goal of the project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

### waypoint translation using spline
. The project works entirely in Frenet coordinate
. The highway is a loop, so the waypoint information at 0 is added back to the end of the waypoint vector, to ensure smooth transition.
. Then Spline are constructed to fit the waypoint for x, y, dx and dy based on s value.
. Translations from Frenet (s, d) coordinate to x, y space, is using the spline constructed to ensure smoothness.

### planner
. The planner will try to keep the car stay in the same lane, if the car in front is far away, and keep the car moving at maximum allowed speed.
. If there is a car in front, and we have to slow down. Then it will look at both the left and right lane, to determine if it will safe to change lane.
. The criteria to determine if it is preferable to change lane, is based on the distance from the car in front and behind in the target lane. The speed of the car in the target lane, and if the target lane is the middle lane.
. When it changes lane, the safe distance will be dependent on the speed of the car in front, and behind. If the car from behind is fast approaching, it should need a larger safe distance before it switches lane.
. The middle lane is always preferred, as it has both left and right lane to consider for lane changing, as compared to both left most or right most lane.

### jerk smoothness
. The project uses the jerk minimizing method mentioned in lecture to calculate the trajectory.
  
