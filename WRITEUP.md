# **Path Planning project** 

## Writeup


**Path Planning Project**

The goals / steps of this project are the following:
In this project, your goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.


[//]: # (Image References)

[image1]: ./images/1.png "straight"
[image2]: ./images/2.png "line changing"


### Path Planning

The following steps were taken in order to implement the path planning:
- iterate through all cars around, given by the sensor fusion and find those that are ahead, on the left, on the right lane according to our lane.
- if we find a car ahead in our lane, then we check if there is a free lane at our left or right side. If so we change the line
- if we can't change the lane and there is a car ahead, then we reduce the speed of the car.
- creating a path planning set of points from previous points and using the transformation to car coordinates, splines to ge the y coordinate.


### Pipeline
## Path Planning
Our behaviour planning model combines an important step where we are looking for the best lane to follow.

![alt][image1]

In order to not exceed the maximal jerk and acceleration we smoothly reduce or augment the speed according to the fact of having a car ahead or not. Particularly, if there is a car ahead we look at the left lane first and check if there is enough space to perform a lane check (40m of distance with both the ahead and behind car) otherwise we look for the right lane according to our lane. If both options fail, we then reduce the speed:

```
for(int i = 0; i < sensor_fusion.size(); ++i){
		// d coordinate (in Frenet coordinate system)
		float d = sensor_fusion[i][6];
		// get the lane number of the car
		int lane = -1;		
		if(d > 0 && lane < 4){
			lane = 0;
		}
		if(d >= 4 && d < 8){
			lane = 1;
		}
		else if(d >= 8){
			lane = 2;
		}

		if(lane == -1){
			continue;
		}

		// get the speed
	    double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
	    double check_speed = sqrt(vx * vx + vy * vy);
	    double check_car_s = sensor_fusion[i][5];
	    //let's look  where is the car in the future
		check_car_s += ((double) prev_size * .02 * check_speed);// if using previous points can project s value out.
		// cout << "car_s="<<car_s << " check_car_s=" << check_car_s << endl;
		if(lane == currentLane){
			carAhead |= check_car_s > car_s && (check_car_s - car_s) < safeDist;	
		}
		else if(lane - currentLane == -1){
			carLeft |= (car_s - safeDist) < check_car_s && (car_s + safeDist) > check_car_s;
		}
		else if(lane - currentLane == 1){
			carRight |= (car_s - safeDist) < check_car_s && (car_s + safeDist) > check_car_s;
		}		
	}

	// behaviour
	bool tooClose = false;
	int bestLane = currentLane;

	if(carAhead){
		cout << "car ahead" << endl;

		if(!carLeft && currentLane > 0){
			bestLane = currentLane - 1;
			cout << "no car left" << endl;
		}
		else if(!carRight && currentLane != 2){
			bestLane = currentLane + 1;
			cout << "no car right" << endl;
		}
		else{
			tooClose = true;
			cout << "too close" << endl;
		}
	}
```

Finally our car is able to drive smoothly more than 4.32 miles without incident and by changing lanes when needed.


![alt text][image2]

 

## Improvements

We could first change the whole model to a more generalized model by using costs. Like a lower cost when there is no car or a lower cost for the fastest lane with enough distance between cars in that lane, lower cost for the center lane when possible, etc.

We could check the heading direction of cars in other lanes in order to prevent collision and start a speed descrease much earlier


