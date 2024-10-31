/*----------------------------------------------------------------------------*/
/*    Author:       Morgan Hu                                                 */
/*----------------------------------------------------------------------------*/

#include "autonomous.h"
#include "task.h"
#include "autopath.h"
#include <iostream>
#undef __ARM_NEON__
#undef __ARM_NEON
#include <eigen-3.4.0/Eigen/Dense>

using namespace vex;
using namespace Eigen;
competition Competition;

void runAuto()
{
  int a = Brain.timer(msec);
  super_intake.set(0);
  puncher_air.set(0);
  inAuto = 1;
  special = 0;
  inManualControl = 0;
  switch (autoRoutine)
  {
  case 0:
    break;
  case 1:
    if (!afraid)
      one();
    else
      one_afraid();
    break;
  case 2:
    two();
    break;
  case 3:
    three();
    break;
  case 4:
    four();
    break;
  default:
    break;
  }
  cout << "time used: " << Brain.timer(msec) - a << endl;
  inAuto = 0;
}

void autonomous(void)
{
  runAuto();
}

void usercontrol(void)
{
  fifteen = 0;
  super_intake.set(0);
  puncher_air.set(0);
  expansion.set(0);
  inAuto = 0;

  while (1)
  {
    // if(BX) test1();
    // movement control for the chassis
    ch_move();
    // activates the intake and the roller
    intakeAndRoll();

    // select auto route
    if (!lRight && RIGHT)
      autoRoutine = (autoRoutine == 4) ? 1 : autoRoutine + 1;
    
    // shoot the disks through the puncher
    if (R1 && !lR1)
      readyToPunch = 1;

    // make the bot auto aim and turn to the goal
    if (R2)
    {
      PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
    }

    // activates the expansion during the last 10 seconds of the match
    if(BY&&!lBY){
      expansion.set(1);
    }

    // enable user to manually control the shooting range of the puncher
    if (BA && !lBA)
    {
      manualPuncherSet = (manualPuncherSet == 2) ? 0 : manualPuncherSet + 1;
    }

    lR1 = R1;
    lBY = BY;
    lUP = UP;
    lRight = RIGHT;
    lDown = DOWN;
    lLEFT = LEFT;
    lBA = BA;
    lBB = BB;
    delay(20);
  }
}

int main()
{
  delay(200);
  Controller1.Screen.clearScreen();

  // initialize the status of the pneumatics
  super_intake.set(0);
  puncher_air.set(0);
  expansion.set(0);
  // task GETGYROREADING(getGyroReading);

  // print essential status on the screen of the controller
  task PRINTSTUFF(printStuff);

  // updating the global orientation of the car & uodating location of the bot using the wheel odometry algorithm
  task UPDATECOR(updateCOR);
  task UPDATEGLOBALROT(updateGlobalRot);

  /* tasks to control the puncher so that the puncher can automatically adjust its shooting range according 
  to the distance between the bot and the goal */
  task SHOOTJUDGER(ShootJudger);
  task PUNCHERENCCONTROL(PuncherEncControl);
  task PUNCHERJUDGER(PuncherJudger);

  // a special task that enables the bot to automatically aim the target even when the user is controlling the movement of the bot
  task GIVESPEEDINAIM(giveSpeedInAim);
  // a special task that enbales the user to switch between three powerful controlling mode for locomotion
  task VAC(vac);
  delay(200);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  while (true)
  {
    delay(100);
  }
}
