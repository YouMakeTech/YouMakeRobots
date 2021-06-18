// Simple code to get started with the YouMakeRobots class
// Connect servo-motors to pin 22, 23, ... 53 of the Arduino MEGA board
// The serial monitor of the Arduino IDE can be used to control them.
// Select a speed of 115200 bauds in the serial monitor.
//
// To move a servo, type "MOVE:<servoId>=<angle>" in the serial monitor
// - <servoId> is the servo id between 0 and 31 included. 
//   0 corresponds to the servo connected to pin 22 and 31 corresponds to
//   the servo connected to pin 53.
// - <angle> is the target angle in degrees, between -90 and +90 included.
//   An increasing angle makes the servo to rotate clockwise, while a 
//   decreasing angle makes the servo to rotate counter-clockwise.
//
// For example, to move the servo connect to pin 22 to +45 degrees, type
// "MOVE:0=45" without the ""
// It is possible to move multiple servos at the same time. 
// For example, to move the first 8 servos to 0 degrees, type
// "MOVE:0=0,1=0,2=0,3=0,4=0,5=0,6=0,7=0" without the ""
// 
// Other commands are available. To see an up to date list of available commands,
// refer to method executeCommand in YouMakeRobots.cpp
//
// For example, "SPEED:1000" makes all moves last 1000 ms. If you want your robot
// to move faster, try "SPEED:200".
// Another useful command is TRIM, which allows to "trim" / adjust the center of each servo.
// by adding a constant offset in us to all commanded pulse widths of this servo.
// Try for example "TRIM:0=40" or "TRIM:0=-40" to offset the center of the servo 
// connected to pin 22 (servo 0) by +40us or -40us.

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
