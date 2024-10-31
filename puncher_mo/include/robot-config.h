#ifndef _CONFIG_
#define _CONFIG_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;
using namespace std;
brain Brain;
#undef __ARM_NEON__
#undef __ARM_NEON
#include <eigen-3.4.0/Eigen/Dense>

using namespace Eigen;

// define essential config
controller Controller1 = controller(primary);

motor intk = motor(PORT1, ratio18_1, 0);
motor roller = motor(PORT13, ratio18_1, 1);
motor LF = motor(PORT20, ratio6_1, 1);
motor LB = motor(PORT5, ratio6_1, 1);
motor RF = motor(PORT4, ratio6_1, 1);
motor RB = motor(PORT6, ratio6_1, 1);
motor leftPuncher = motor(PORT8, ratio36_1, 0);
motor rightPuncher = motor(PORT12, ratio36_1, 1);

encoder xpos = encoder(Brain.ThreeWirePort.A);
encoder ypos = encoder(Brain.ThreeWirePort.C);
inertial Gyro = inertial(PORT11);
//gyro v4Gyro = gyro(Brain.ThreeWirePort.G);4
digital_out expansion = digital_out(Brain.ThreeWirePort.H);
digital_out super_intake = digital_out(Brain.ThreeWirePort.E);
digital_out puncher_air = digital_out(Brain.ThreeWirePort.F);

digital_out test_lag_1 = digital_out(Brain.ThreeWirePort.G);
digital_out test_lag_2 = digital_out(Brain.ThreeWirePort.F);
void vexcodeInit(void);

#endif