#ifndef SDC_BESTLANE_H
#define SDC_BESTLANE_H

#include <unordered_map>
#include <stdexcept> 
#include <climits>
#include <tuple>
#include <vector>

using namespace std;

tuple<int, bool> bestChangeLaneOption(const vector<vector<double>> &sensor_fusion, int myLane, int indexToExclude, 
			int prev_size, double car_s, double ref_vel);

tuple<int, bool>  changeToLane(const vector<vector<double>> &sensor_fusion, int currentLane, int prev_size, double car_s);
#endif