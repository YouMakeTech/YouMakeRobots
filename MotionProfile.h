// header file for the MotionProfile class.
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

#ifndef MOTIONPROFILE_H
#define MOTIONPROFILE_H

#include <Arduino.h>

#define CONSTANT_SPEED_PROFILE 1
#define TRIANGULAR_SPEED_PROFILE 2

class MotionProfile
{
  public:

    /* Variables */
    unsigned int type;
    unsigned long startTime;
    unsigned int duration;
    float startValue;
    float finalValue;
  

    /* Methods */
    unsigned long endTime();
    void constantSpeedProfile(float _startValue,float _finalValue,unsigned int _duration);
    void triangularSpeedProfile(float _startValue,float _finalValue,unsigned int _duration);
    float getValueAtTime(unsigned long _time);
    float getValueNow();
    bool isComplete();

  private:
    float constantSpeedIntepolation(unsigned long _time);
    float triangularSpeedIntepolation(unsigned long _time);

};

#endif /* MOTIONPROFILE_H */
