// Sketch to control the Zombie robot
// The Zombie robot is a walking droid controlled by 8 servo motors.
//
// Servo Motors location (Front View)
// 
// =(1)=(0)=====(4)=(5)=
//         =====
//        (2) (6)
//         |   |
//        (3) (7)

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
  hello();
}

void loop() {
  walk();

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

void hello() {
  robot.executeCommand("SPEED:200");
  robot.executeCommand("MOVE:1=-60,5=60");
  robot.executeCommand("MOVE:0=90");
  robot.executeCommand("MOVE:4=-90");
}

void walk(){
  robot.executeCommand("SPEED:900");
  
  // Lower the 2 arms
  robot.executeCommand("MOVE:1=-60,5=60");
  robot.executeCommand("MOVE:0=45,4=-45");

  // Lift the left foot
  robot.executeCommand("MOVE:3=-10,7=-20");

  // Move the left foot forward
  robot.executeCommand("MOVE:2=-30,6=-30");

  // Put the left foot back on the floor + lower the left arm
  robot.executeCommand("MOVE:3=0,7=0");

  // Lift the right foot
  robot.executeCommand("MOVE:7=10,3=20");

  // Move the right foot
  robot.executeCommand("MOVE:2=30,6=30");

  // Put the right foot back on the floor
  robot.executeCommand("MOVE:7=0,3=0");
}
