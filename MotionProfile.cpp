// Implementation of the MotionProfile class.
// The MotionProfile classprovides interpolation methods to move smoothly 
// from a start value to an end value in a specified duration.
// This is used by the YouMakeRobots class to move smoothly servo-motors.
// Currently, 2 type of interpolations/profiles are implemented:
// - constant speed: See method constantSpeedIntepolation
// - triangular speed: See method triangularSpeedProfile

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

#include "MotionProfile.h"

/* unsigned long endTime() returns the expect end time in ms for the motion profile
/* the end time is returned as the number of milliseconds passed since the Arduino 
 *  board began running the current program.
 */
unsigned long MotionProfile::endTime() {
  return(startTime+duration);
}

/* void constantSpeedProfile(float _startValue,float _finalValue,unsigned int _duration) 
 *  specifies a motion at constant speed from a start value of _startValue 
 *  to a final value of _finalValue. The time allowed for the move is specified by 
 *  _duration (in ms)
 */
void MotionProfile::constantSpeedProfile(float _startValue,float _finalValue,unsigned int _duration) {
  type=CONSTANT_SPEED_PROFILE;
  startTime=millis();
  startValue=_startValue;
  finalValue=_finalValue;
  duration=_duration;

}

/* triangularSpeedProfile(float _startValue,float _finalValue,unsigned int _duration)
 *  specifies a motion with triangular a speed profile.
 *  During the first half of the move (between start time and (start time+end time)/2), 
 *  accelerate at constant acceleration from a speed of zero to maxSpeed
 *  During the 2nd half of the move, decelerate at constant acceleration back to zero
 */
void MotionProfile::triangularSpeedProfile(float _startValue,float _finalValue,unsigned int _duration) {
  type=TRIANGULAR_SPEED_PROFILE;
  startTime=millis();
  startValue=_startValue;
  finalValue=_finalValue;
  duration=_duration;
}

/* float getValueAtTime() returns the value of the motion profile at the specified time */
float MotionProfile::getValueAtTime(unsigned long _time) {
  float value;
  if(_time<=startTime) {
    value=startValue;
  } else if(_time>=endTime()) {
  value=finalValue;
  } else {
    if(type==CONSTANT_SPEED_PROFILE){
      value=constantSpeedIntepolation(_time);
    } else if (type==TRIANGULAR_SPEED_PROFILE){
      value=triangularSpeedIntepolation(_time);
    } else {
      value=finalValue;
    }
  }
  return(value);
}


/* float getValueNow() returns the value of the motion profile for the current time */
float MotionProfile::getValueNow() {
  return(getValueAtTime(millis()));
}

/* isComplete returns true when the motion profile is complete */
bool MotionProfile::isComplete() {
  unsigned long endTime;
  bool isComplete=false;
  endTime=startTime+duration;
  if(millis()>endTime) isComplete=true;
  return(isComplete);
}


float MotionProfile::constantSpeedIntepolation(unsigned long _time) {
  float value;
  
  value=startValue+((_time-startTime)*(finalValue-startValue))/duration;
  return(value);
}

/* During the first half of the move (between start time and (start time+end time)/2), 
 *  accelerate at constant acceleration from a speed of zero to maxSpeed
 *  During the 2nd half of the move, decelerate at constant acceleration back to zero
 */
float MotionProfile::triangularSpeedIntepolation(unsigned long _time) {
  float value;
  float averageSpeed,maxSpeed,acceleration;
  unsigned long halfWayTime;
  averageSpeed=1000*(finalValue-startValue)/duration;  // per s
  maxSpeed=2*averageSpeed;
  halfWayTime=startTime+duration/2;
  acceleration=maxSpeed/(0.5*duration);
  if(_time<=halfWayTime) {
    value=startValue+0.5*(acceleration*(_time-startTime)*(_time-startTime))/1000;
  } else if(_time>halfWayTime) {
    value=finalValue-0.5*(acceleration*(endTime()-_time)*(endTime()-_time))/1000;
  }
  return(value);
}
