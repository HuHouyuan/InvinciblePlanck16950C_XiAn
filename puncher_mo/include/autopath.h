#ifndef AUTOPATH
#define AUTOPATH

#include "robot-config.h"
#include "declaration.h"
#include "autonomous.h"

void test1(){
  timerForward(80, 3000);
  delay(10);
}

void test2(){
  timerForward(-80, 3000);
  delay(10);
}

void test3(){
  timerHorizontal(80, 3000);
  delay(10);
}

void test4(){
  timerHorizontal(-80, 3000);
  delay(10);
}

void one(){

  inAuto = 1;
  fifteen = 1;
  //cout<<":("<<endl;51
  // rx = -95.83;
  // ry = -145.7;

  // rx = -86.11;
  // ry = -145.7;
  //first group
  float fifteenTime = Brain.timer(msec);
  intake_setspeed(100);
  // Odom_move(-113, -138, 125);
  EncoderForward(60, 200);
  Odom_move(-90, -150, 90);
  intake_setspeed(0);
  roller_setspeed(-80);
  driveForward(-40);
  delay(300);
  
  timerForward(100, 200);
  intake_setspeed(100);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  delay(200);
  readyToPunch = 1;
  delay(850);
  
  
  //
  super_intake.set(1);
  Odom_move(-70, -120, 120);
  delay(200);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(-90, -90)));
  EncoderForward(60, 300);
  super_intake.set(0);
  intake_setspeed(100);
  delay(1300);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  readyToPunch = 1;
  delay(850);


  //cout<<"1 finished"<<endl;
  //second group
  super_intake.set(1);
  Odom_move(-55, -100, 45);
  delay(200);
  //PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(-30, -90)));
  EncoderForward(50,100);
  super_intake.set(0);
  intake_setspeed(100);
  delay(1300);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  intake_setspeed(0);
  delay(200);
  readyToPunch = 1;
  delay(850);

  
  //thrid group

  //Odom_move(34,-127, 60);
  //PIDGyroTurn(-180);
  intake_setspeed(100);
  Odom_move(32,-125,60);
  // Odom_move(34,-120,60);
  // Odom_move(37,-90,65);
  Odom_move(32,-60,50);
  Odom_move(20, -30,135);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  readyToPunch = 1;
  // delay(850);
  inAuto = 0;
  fifteen = 0;
  cout<<"timeused:"<<fifteenTime<<endl;
}

void one_afraid(){
  cout<<"inPanic"<<endl;
  inAuto = 1;
  fifteen = 1;
  //cout<<":("<<endl;51
  // rx = -95.83;
  // ry = -145.7;

  // rx = -86.11;
  // ry = -145.7;
  //first group
  float fifteenTime = Brain.timer(msec);
  intake_setspeed(100);
  // Odom_move(-113, -138, 125);
  EncoderForward(60, 200);
  Odom_move(-90, -150, 90);
  intake_setspeed(0);
  roller_setspeed(-40);
  driveForward(-40);
  delay(300);
  
  roller_setspeed(-80);
  delay(400);

  timerForward(100, 200);
  intake_setspeed(100);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  delay(200);
  readyToPunch = 1;
  delay(850);
  
  
  
  // super_intake.set(1);
  
  Odom_move(-78, -112, 120);
  intake_setspeed(-100);
  delay(500);
  //PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(-90, -90)));
  // EncoderForward(60, 300);
  // super_intake.set(0);
  // intake_setspeed(100);
  // delay(1300);
  // PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  // readyToPunch = 1;
  // delay(850);


  //cout<<"1 finished"<<endl;
  //second group
  intake_setspeed(100);
  super_intake.set(1);
  Odom_move(-55, -100, 45);
  delay(200);
  //PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(-30, -90)));
  EncoderForward(50,100);
  super_intake.set(0);
  intake_setspeed(100);
  delay(1300);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  intake_setspeed(0);
  delay(200);
  readyToPunch = 1;
  delay(850);

  
  //thrid group

  //Odom_move(34,-127, 60);
  //PIDGyroTurn(-180);
  intake_setspeed(100);
  Odom_move(32,-125,60);
  // Odom_move(34,-120,60);
  // Odom_move(37,-90,65);
  Odom_move(32,-60,50);
  Odom_move(20, -30,135);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, Vector2f(shootPoint[0]+5, shootPoint[1]-5)));
  readyToPunch = 1;
  // delay(850);
  inAuto = 0;
  fifteen = 0;
  cout<<"timeused:"<<fifteenTime<<endl;
}

void two(){
  manualPuncherSet = 0;
  //cout<<"in two" <<endl;
  inAuto = 1;
  rx = 135.25;
  ry = 34.3;
  fifteen = 1;
  //first group
  intake_setspeed(100);
  Odom_move(140, 110, 135);
  delay(300);
  Odom_move(155, 100, 180);
  roller_setspeed(-1);
  timerForward(-60, 400);
  roller_setspeed(-100);
  delay(400);

  timerForward(100,150);
  intake_setspeed(100);
  delay(600);
  //Odom_move(150, 100, 135);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  
  readyToPunch = 1;
  delay(850);

  //second group
  super_intake.set(1);
  Odom_move(120, 90, 135);
  delay(200);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(92.5,92.5)));
  EncoderForward(60, 160);
  intake_setspeed(100);
  super_intake.set(0);
  delay(2000);

  EncoderForward(-60, 150);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  readyToPunch = 1;
  delay(850);

  //third group
  intake_setspeed(100);
  Odom_move(100,40, 230);
  delay(200);
  //PIDGyroTurn(-35);
  Odom_move(40, -10, 230);
  // Odom_move(30, -5, 230);
  delay(200);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  delay(800);
  readyToPunch = 1;
  delay(850);

  // forth group
  // intake_setspeed(100);
  
  // //Odom_move(30, -20, 310);
  // //delay(400);
  // Odom_move(60, -28, 315);
  // //Odom_move(90, -25, 305);
  // Odom_move(120, -28, 325);
  // // Odom_move(90, 0, 147);
  // PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  // delay(600);
  // readyToPunch = 1;
  // //delay(850);
  // inAuto = 0;
  fifteen = 0;
}

void three(){
  inAuto = 1;
  fifteen = 1;
  //cout<<":("<<endl;51
  // rx = -95.83;
  // ry = -145.7;
  rx = -86.11;
  ry = -145.7;
  //first group
  Odom_move(-100, -145, 90);
  driveForward(-40);
  delay(300);
  driveForward(-2);
  EncoderRoll(-100, 100);
  timerForward(100, 200);
  intake_setspeed(100);
  Odom_move(-113, -138, 125);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  delay(500);
  readyToPunch = 1;
  delay(850);
  
  
  //cout<<"1 finished"<<endl;
  //second group
  super_intake.set(1);
  Odom_move(-55, -100, 45);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(-30, -90)));
  EncoderForward(50,200);
  super_intake.set(0);
  intake_setspeed(100);
  delay(1200);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  intake_setspeed(0);
  delay(200);
  readyToPunch = 1;
  delay(850);

  //third
  intake_setspeed(100);
  Odom_move(0, -60, 45);
  Odom_move(90, 30, 45);
  Odom_move(150, 115, 180);
  timerForward(-100, 200);
  driveForward(-2);
  EncoderRoll(100, 95);
  timerForward(100, 200);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  intake_setspeed(0);
  readyToPunch = 1;
  delay(850);
  inAuto = 0;
  fifteen = 0;
}

void four(){
  inAuto = 1;
  fifteen = 0;
  ManualShoot = 1;
  chooseBlue = 1;
  rx = 135.25;
  ry = 34.3;
  //first group
  intake_setspeed(100);
  Odom_move(135, 105, 135);
  Odom_move(160, 115, 180);
  timerForward(-50, 200);
  driveForward(-2);
  EncoderRoll(60, 20);
  intake_setspeed(100);
  timerForward(100,100);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  delay(600);
  readyToPunch = 1;
  delay(850);

  //second || 3die
  super_intake.set(1);
  Odom_move(115,115, 210);
  delay(200);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(90, 90)));
  EncoderForward(30, 200);
  intake_setspeed(100);
  super_intake.set(0);
  delay(1500);

  Odom_move(100, 160, 270);
  timerForward(-100, 200);
  driveForward(-2);
  EncoderRoll(60, 20);
  timerForward(100, 100);  
  Odom_move(62, 110, 225);
  delay(200);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  readyToPunch = 1;
  delay(850);

  //third || 3die
  super_intake.set(1);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(30,90)));
  intake_setspeed(100);
  EncoderForward(60, 400);
  super_intake.set(0);
  delay(1500);

  Odom_move(-30, 150, 180);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  readyToPunch = 1;
  delay(850);

  //forth || kan
  Odom_move(-39, 120, 240);
  Odom_move(-39, 60, 250);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  delay(800);
  readyToPunch = 1;
  delay(850);

  //five || kan
  Odom_move(-60, 33, 150);
  Odom_move(-120, 33, 150);
  
  Odom_move(-120, 35, 160);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  delay(600);
  readyToPunch = 1;
  delay(850);

  //six
  intake_setspeed(100);



  chooseRed = 1;
  chooseBlue = 0;
  Odom_move(-90, -30+15, 270);
  Odom_move(-130, -60, 270);
  Odom_move(-120+15, -120, 255);
  delay(200);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(-150, -150)));
  EncoderForward(50, 500);
  //roller 2nd
  delay(200);
  Odom_move(-155, -105, 360);
  timerForward(-100, 200);
  driveForward(-2);
  EncoderRoll(60, 20);
  intake_setspeed(100);
  timerForward(100,200);
  delay(300);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  readyToPunch = 1;
  delay(850);

  //seven
  super_intake.set(1);
  Odom_move(-115,-115, 210+360);
  delay(200);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(90, 90)));
  EncoderForward(30, 200);
  intake_setspeed(100);
  super_intake.set(0);
  delay(1500);

  Odom_move(-100, -130, 450);
  timerForward(-100, 400);
  driveForward(-2);
  EncoderRoll(60, 20);
  timerForward(100, 100);


  Odom_move(-65, -110, 45+360);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  readyToPunch = 1;
  delay(850);


  //eight
  super_intake.set(1);
  Odom_move(-55, -100, 45+360);
  delay(200);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, Vector2f(-30, -90)));
  EncoderForward(60, 200);
  super_intake.set(0);
  intake_setspeed(100);
  delay(1500);

  Odom_move(35, -135, 0+360);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  readyToPunch = 1;
  delay(850);

  //eight
  Odom_move(27, -120, 60+360);
  Odom_move(27, -60, 70+360);
  delay(200);
  PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
  delay(300);
  readyToPunch = 1;
  delay(850);

  

  //nine
  Odom_move(60, -25, 300);
  Odom_move(120, -25, 310);
  delay(200);
  PIDGyroTurn(mygyro.rotation() + LookAt(0, shootPoint));
  delay(300);
  readyToPunch = 1;
  delay(850);
  notBad = 1;

  Odom_move(120, 120, 225+360);
  expansion.set(1);
}

void five(){
  inAuto = 1;
  rx = 135.25;
  ry = 34.3-60;
  delay(1400);
}
#endif