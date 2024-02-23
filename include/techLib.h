#include "main.h"
#include <fstream>
#include <complex.h>
#include "lemlib/api.hpp"
#include <iostream>
#include <chrono>
#include <cmath>
#include <future>
#include <atomic>
#include <mutex>

#pragma

const int LEFT_MTR_B = 1;
const int LEFT_MTR_M = 12;
const int LEFT_MTR_F = 11;
const int RIGHT_MTR_B = 8;
const int RIGHT_MTR_M = 19;
const int RIGHT_MTR_F = 20;
const int CATA_R = 2; //left reversed
const int CATA_F = 9; //right not reversed
const int ROTATION_V = 18; //vertical rotation sensor
const int ROTATION_H = 19; //horizontal rotation sensor
const int INERTIAL = 10;


