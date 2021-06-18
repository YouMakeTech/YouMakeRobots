// header file for the YouMakeRobots class.
// The YouMakeRobots class provides the main interface to control robots for the user.
// - Control up to 32 servo-motors connected between pin 22 and 53 of the Arduino MEGA board.
// - Move servo motors smoothly using a triangular speed profile (constant acceleration/decceleration) 
//   The triangular speed profile is implemented in class MotionProfile.
// - Allow to configure each servo individually (min. pulsewidth, max. pulsewidth,
//   center pulsewidth and trim)
// - Provide a command interface that can be used to interactively control a robot using the Serial Monitor
//   or the serial line

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

#ifndef YOUMAKEROBOTS_H
#define YOUMAKEROBOTS_H

#include <Arduino.h>
#include <Servo.h>
#include "MotionProfile.h"

#define NUM_SERVOS     32  // the number of servos (maximum 32)
#define SERVO_MIN     640  // the pulse width, in microseconds, corresponding to the minimum (-90 degree) angle on the servo
#define SERVO_CENTER 1500  // the pulse width, in microseconds, corresponding to the minimum (0 degree) angle on the servo
#define SERVO_MAX    2400  // the pulse width, in microseconds, corresponding to the maximum (+90 degree) angle on the servo

class YouMakeRobots
{
  public:
    void init(); 
    void initialPosition();
    
    void setServoMin(unsigned int servoNum, unsigned int us);
    void setServoMax(unsigned int servoNum, unsigned int us);
    void setServoCenter(unsigned int servoNum, unsigned int us);
    void setServoTrim(unsigned int servoNum, int us);
    unsigned int getServoMin(unsigned int servoNum);
    unsigned int getServoMax(unsigned int servoNum);
    unsigned int getServoCenter(unsigned int servoNum);
    int getServoTrim(unsigned int servoNum);
    unsigned int angleToPulse(unsigned int servoNum,float angle);
    float pulseToAngle(unsigned int servoNum,unsigned int pulse);
    void writePulse(unsigned int servoNum,unsigned int pulse);
    void writeAngle(unsigned int servoNum,float targetPosition);
    
    void moveTo(float targetPositionArray[],unsigned int speed);
    void executeCommand(String s);

  private:
    Servo servo[NUM_SERVOS];
    float servoPosition[NUM_SERVOS];      // Current servo positions (array of 32 angles, between -90 and 90 degrees)
    unsigned int servoMin[NUM_SERVOS];    // Pulse width in us corresponding to -90 degrees
    unsigned int servoCenter[NUM_SERVOS]; // Pulse width in us corresponding to 0 degrees
    unsigned int servoMax[NUM_SERVOS];    // Pulse width in us corresponding to +90 degrees
    int servoTrim[NUM_SERVOS];            // Trim in us added to all commanded pulse widths
    unsigned int __moveSpeed;           // Default speed for MOVE commands (used by executeCommand)
    void __executeMOVE__(String cmd);
    void __executeTRIM__(String cmd);
    void __executeSPEED__(String cmd);
    
};

#endif /* YOUMAKEROBOTS_H */
