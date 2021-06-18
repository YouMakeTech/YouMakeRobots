// Implementation of the YouMakeRobots class.
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

#include "YouMakeRobots.h"

void YouMakeRobots::init() {
 
  /* attach servos to the pins. First servo is connected to pin 22 
     of the Arduino Mega 2560 board,  second servo to pin 23 and so on.
     Up to 32 servos can be connected to the Arduino Mega 2560 board,
     from pin 22 to pin 53.
   */
  for(unsigned int servoNum=0;servoNum<NUM_SERVOS;servoNum++){
    servoPosition[servoNum]=0;
    servoMin[servoNum]=SERVO_MIN;
    servoCenter[servoNum]=SERVO_CENTER;
    servoMax[servoNum]=SERVO_MAX;
    servoTrim[servoNum]=0;
    __moveSpeed=1000; 
    servo[servoNum].attach(22+servoNum, servoMin[servoNum], servoMax[servoNum]);
  }
  initialPosition();
}


/* initialPosition() move all servos to 0 degrees */
void YouMakeRobots::initialPosition() {
  float targetPosition[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  unsigned int servoNum;
  for(servoNum=0;servoNum<NUM_SERVOS;servoNum++) {
    writeAngle(servoNum,targetPosition[servoNum]);
  }
  delay(1000);
}

void YouMakeRobots::setServoMin(unsigned int servoNum, unsigned int us) {
  if(servoNum<NUM_SERVOS) {
    servoMin[servoNum]=us;
  }
  Serial.print("servoMin:");
  for(unsigned int i=0;i<NUM_SERVOS;i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(servoMin[i]);
    Serial.print(",");
  }
  Serial.println("");
}

void YouMakeRobots::setServoMax(unsigned int servoNum, unsigned int us) {
  if(servoNum<NUM_SERVOS) {
    servoMax[servoNum]=us;
  }
  Serial.print("servoMax:");
  for(unsigned int i=0;i<NUM_SERVOS;i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(servoMax[i]);
    Serial.print(",");
  }
  Serial.println("");
}
    
void YouMakeRobots::setServoCenter(unsigned int servoNum, unsigned int us) {
  if(servoNum<NUM_SERVOS) {
    servoCenter[servoNum]=us;
  }
  Serial.print("servoCenter:");
  for(unsigned int i=0;i<NUM_SERVOS;i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(servoCenter[i]);
    Serial.print(",");
  }
  Serial.println("");
}

void YouMakeRobots::setServoTrim(unsigned int servoNum, int us) {
  if(servoNum<NUM_SERVOS) {
    servoTrim[servoNum]=us;
    writeAngle(servoNum, servoPosition[servoNum]);
  }
  Serial.print("servoTrim:");
  for(unsigned int i=0;i<NUM_SERVOS;i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(servoTrim[i]);
    Serial.print(",");
  }
  Serial.println("");
}

unsigned int YouMakeRobots::getServoMin(unsigned int servoNum) {
  unsigned int us=SERVO_MIN;
  if(servoNum<NUM_SERVOS) {
    us=servoMin[servoNum];
  }
  return(us);
}

unsigned int YouMakeRobots::getServoMax(unsigned int servoNum) {
  unsigned int us=SERVO_MAX;
  if(servoNum<NUM_SERVOS) {
    us=servoMax[servoNum];
  }
  return(us);
}

unsigned int YouMakeRobots::getServoCenter(unsigned int servoNum) {
  unsigned int us=SERVO_CENTER;
  if(servoNum<NUM_SERVOS) {
    us=servoCenter[servoNum];
  }
  return(us);
}

int YouMakeRobots::getServoTrim(unsigned int servoNum) {
  unsigned int us=0;
  if(servoNum<NUM_SERVOS) {
    us=servoTrim[servoNum];
  }
  return(us);
}

/* unsigned int pulse=angleToPulse(unsigned int servonum, float angle) 
 *  convert an angle to  a pulse duration 
 *  - servonum is the servo number between 1 and 32
 *  - angle is a float between -90 degree and +90 degree
 *  - pulse is the pulse width in us to send to the servo
 */
unsigned int YouMakeRobots::angleToPulse(unsigned int servoNum,float angle) {
  unsigned int pulse;
  if(angle<0) {
    pulse=(unsigned int)map(angle,-90.0,0.0,servoMin[servoNum],servoCenter[servoNum]);
  } else {
    pulse=(unsigned int)map(angle,0.0,90.0,servoCenter[servoNum],servoMax[servoNum]);  
  }
  pulse=pulse+servoTrim[servoNum];
  return(pulse);
}

float YouMakeRobots::pulseToAngle(unsigned int servoNum,unsigned int pulse) {
  float angle;
  pulse=pulse-servoTrim[servoNum];
  if(pulse<servoCenter[servoNum]) {
    angle=(float)map(pulse,servoMin[servoNum],servoCenter[servoNum],-90.0,0.0);
  } else {
    angle=(float)map(pulse,servoCenter[servoNum],servoMax[servoNum],0.0,90.0);
  }
  return(angle);
}

void YouMakeRobots::writePulse(unsigned int servoNum,unsigned int pulse) {
  servo[servoNum].writeMicroseconds(pulse);
}

void YouMakeRobots::writeAngle(unsigned int servoNum, float angle) {
  unsigned int pulse;
  if(servoNum<NUM_SERVOS && angle>=-90.0 && angle<=90.0) {
    pulse=angleToPulse(servoNum,angle);
    servo[servoNum].writeMicroseconds(pulse);
    servoPosition[servoNum]=angle;
  }
}

void YouMakeRobots::moveTo(float targetPosition[],unsigned int speed) {
  float targetSpeed[NUM_SERVOS];
  float startPosition[NUM_SERVOS];
  unsigned int startPulse[NUM_SERVOS],currentPulse[NUM_SERVOS],lastPulse[NUM_SERVOS],targetPulse[NUM_SERVOS];
  unsigned long startTime,currentTime;
  float deltaPulse;
  MotionProfile motionProfile[NUM_SERVOS];
  
  unsigned int i,iteration;
  bool targetReached;

  // Initialize lastPulse
  for(i=0;i<NUM_SERVOS;i++) {
    lastPulse[i]=0;
  }

  // Record servos initial positions & pulse lengths
  for(i=0;i<NUM_SERVOS;i++) {
    startPosition[i]=servoPosition[i];
    startPulse[i]=angleToPulse(i,startPosition[i]);
  }

  // Keep current servo position when targetPosition[i] is outside of [-90 90] degrees
  //Serial.print("targetPosition=[");
  for(i=0;i<NUM_SERVOS;i++) {
    if(targetPosition[i]<-90.0 || targetPosition[i]>90.0) {
      targetPosition[i]=servoPosition[i];
    }
    //Serial.print(targetPosition[i]);
    //Serial.print(",");
  }
  //Serial.println("]");

  // Compute target pulse lengths from target positions
  for(i=0;i<NUM_SERVOS;i++) {
    targetPulse[i]=angleToPulse(i,targetPosition[i]);
  }

  if (speed>10) {
    /* follow a motion profile from startPulse to targetPulse
     * such that the move last the specified duration in us
     */
    for (i=0;i<NUM_SERVOS;i++) {
      motionProfile[i].triangularSpeedProfile(startPulse[i],targetPulse[i],speed);
    }

    while(true) {
      currentTime=millis();
      //Serial.print("currentPulse=[");
      for (i=0;i<NUM_SERVOS;i++) {
        currentPulse[i]=motionProfile[i].getValueAtTime(currentTime); // interpolate pulse at current time from motion profile
        //Serial.print(currentPulse[i]);
        //Serial.print(",");

        if(currentPulse[i]!=lastPulse[i]) {
          writePulse(i,currentPulse[i]);
        }
        lastPulse[i]=currentPulse[i];
      }
      //Serial.println("]");

      targetReached=true;
      for (i=0;i<NUM_SERVOS;i++) {
        if(targetPulse[i]!=currentPulse[i]) {
          targetReached=false;
          break;
        }
      }

      if(targetReached) {
        // All servos have reached their target angles
        break;
      }
    }
  
  }

  Serial.print("servoPosition:");
  for (i=0;i<NUM_SERVOS;i++) {
    writePulse(i,targetPulse[i]);
    servoPosition[i]=targetPosition[i];
    Serial.print(i);
    Serial.print("=");
    Serial.print(servoPosition[i]);
    Serial.print(",");
  }
  Serial.println("");
    
}

/* void executeCommand(String cmd) allows to interact with the robot by sending commands 
 *  from the serial monitor
 *  "MOVE:0=45,2=-90" move servo 0 to 45 degrees and servo 2 to -90 degrees
 *  "TRIM:1=20" trim servo 1 by adding 20 us to all pulse widths commanded to this servo
 */
void YouMakeRobots::executeCommand(String cmd) {
  cmd.toUpperCase();
  Serial.print("> ");
  Serial.println(cmd);
  if(cmd.startsWith("MOVE:")) {
      cmd.remove(0,5);
      __executeMOVE__(cmd);
  } else if(cmd.startsWith("TRIM:")) {
      cmd.remove(0,5);
      __executeTRIM__(cmd);
  } else if(cmd.startsWith("SPEED:")) {
      cmd.remove(0,6);
      __executeSPEED__(cmd);
  } else {
    Serial.println("Unrecognized command");
  }
  
}

void YouMakeRobots::__executeMOVE__(String cmd) {
  unsigned int servoNum=255;
  float targetPosition=255;
  float targetPositionArray[32]={255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};
  unsigned int i;
  unsigned int r=0;
  for(i=0;i<=cmd.length();i++) {
    if(cmd.charAt(i)==',' || cmd.charAt(i)=='=' || i==cmd.length()) {
      if(cmd.charAt(i)=='=') {
        // the number before "=" is the servo id between 0 and 31
        servoNum=cmd.substring(r,i).toInt();
        r=i+1;
      }
      else if(cmd.charAt(i)==',' || i==cmd.length()) {
        // the number before a "," or the last number of a string is 
        // the target angle between -90 and 90 degrees 
        targetPosition=cmd.substring(r,i).toFloat();
        r=i+1;
      }
    }
    if(servoNum>=0 && servoNum<NUM_SERVOS && targetPosition>=-90 && targetPosition<=90) {
      // we got a servo number and a target position
      // => store the target position in targetPositionArray
      // and get ready for the next servo
      targetPositionArray[servoNum]=targetPosition;
      servoNum=255;
      targetPosition=255.0;
    }
  }
  
  // Move all servos to their target positions
  moveTo(targetPositionArray,__moveSpeed);
}

void YouMakeRobots::__executeTRIM__(String cmd) {
  unsigned int servoNum=255;
  int servoTrim=10000;
  unsigned int i;
  unsigned int r=0;
  for(i=0;i<=cmd.length();i++) {
    if(cmd.charAt(i)==',' || cmd.charAt(i)=='=' || i==cmd.length()) {
      if(cmd.charAt(i)=='=') {
        // the number before "=" is the servo id between 0 and 31
        servoNum=cmd.substring(r,i).toInt();
        r=i+1;
      }
      else if(cmd.charAt(i)==',' || i==cmd.length()) {
        // the number before a "," or the last number of a string is 
        // the trim in us 
        servoTrim=cmd.substring(r,i).toInt();
        r=i+1;
      }
    }
    
    if(servoNum>=0 && servoNum<NUM_SERVOS && servoTrim>-1000 && servoTrim<1000) {
      // we got a servo number and a desired trim
      // and get ready for the next servo
      setServoTrim(servoNum,servoTrim);
      servoNum=255;
      servoTrim=10000;
    }
  }
}

void YouMakeRobots::__executeSPEED__(String cmd) {
  __moveSpeed=cmd.toInt();
}
