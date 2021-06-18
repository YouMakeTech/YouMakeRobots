// Sketch to control a quadruped robot
// The quadruped robot is a walking robot controlled by 8 servo motors.
//
// Servo Motors location (Top View)
//
//         +---+
// =(7)=(3)|   |(0)=(4)=
//         |   |
// =(6)=(2)|   |(1)=(5)=
//         +---+
//
//   * Servo 0 = Front Right     (pin 22)
//   * Servo 1 = Rear Right      (pin 23)
//   * Servo 2 = Rear Left       (pin 24)
//   * Servo 3 = Front Left      (pin 25)
//   * Servo 4 = Front Right Leg (pin 26)
//   * Servo 5 = Front Right Leg (pin 27)
//   * Servo 6 = Front Right Leg (pin 28)
//   * Servo 7 = Front Right Leg (pin 29)

// This file is part of YouMakeRobots.
// Copyright (C) 2020 Vincent Mistler (YouMakeTech)
// For conditions of distribution and use, see copyright notice in LICENCE

// YouMakeRobots is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// YouMakeRobots is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with YouMakeRobots.  If not, see <http://www.gnu.org/licenses/>.


#include "YouMakeRobots.h"

YouMakeRobots robot;


void setup() {
  Serial.begin(115200);
  
  robot.init();
  robot.initialPosition();
  robot.executeCommand("TRIM:0=40,1=-40,2=-40,3=40,4=-10,5=-50,6=0,7=-40");
  walk(5);
}

void loop() {
   
}

void serialEvent() {
  /* Allow to send commands to the robot using the serial monitor
    Commands allow to change servo positions, change servo trims, 
    store current position of the robot in memory, and go back to 
    a previously saved position. See method executeCommand in class
    Robot for more details
  */ 
  String cmd;
  cmd=Serial.readString();
  robot.executeCommand(cmd);
}

void walk(unsigned int n) {
  /* Walk  n steps using Creep Gait
   * See https://makezine.com/2016/11/22/robot-quadruped-arduino-program/
   * and "Walking Pattern for Quadruped as Observer Robot" 
   * Nuril Esti Khomariah and Samsul Huda 2019 J. Phys.: Conf. Ser. 1201 012010
   * for more details
   */
  for(unsigned int i=0;i<n;i++) {
    // Starting position for walking
    robot.executeCommand("MOVE:0=0,1=0,2=45,3=-45,4=0,5=0,6=0,7=0");
    
    // The front-right leg lifts up and reaches out, far ahead of the robot
    robot.executeCommand("MOVE:0=70,4=45");
      
    // All the legs shift backward, moving the body forward
    robot.executeCommand("MOVE:0=45,1=-45.00,2=70,3=0,4=0");
  
    // The back-left leg lifts and steps forward alongside the body. This position is the mirror image of the starting position.
    robot.executeCommand("MOVE:2=0,6=45");
    robot.executeCommand("MOVE:6=0");
  
    // The front-left leg lifts and reaches out, far ahead of the robot.
    robot.executeCommand("MOVE:3=-70,7=-45");
    
    // Again, all the legs shift backward, moving the body forward.
    robot.executeCommand("MOVE:0=0,1=-70,2=45,3=-45,7=0");
  
    // Move the rear right leg back to starting position
    robot.executeCommand("MOVE:1=0,5=-45");
    robot.executeCommand("MOVE:5=0");
    
    // We are back to the starting position
  }
}
