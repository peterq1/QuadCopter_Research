//
//  PID.h
//  PID
//
//  Created by Yosub Lee & Peter Quan on 7/15/14.
//  Copyright (c) 2014 Yosub Lee & Peter Quan. All rights reserved.
//
 
#ifndef PID_CONTROL
#define PID_CONTROL
 
#include <iostream>
#include <cmath>
#include <chrono>
#include <ctime>
 
using namespace std;
 
class PID {
public:
    PID();
    ~PID();
    void setPoint(double point);
    void setGains(double kp, double ki, double kd);
    double compute(double input);
    void setTunings(double kp2, double ki2, double kd2);
    void setSampleTime(double newSampleTime);
    void setOutputLimits(double min, double max);
    
private:
    double m_input, m_output, m_setpoint;
    double m_iTerm, m_lastInput;
    double kp, ki, kd;
    double m_sampleTimeSec;
    double m_ratio;
    bool initialized;   
    double m_sampleTime;	// AHRS sample time
    chrono::time_point<std::chrono::high_resolution_clock> m_now, m_lastTime;
    double m_outMin, m_outMax;
};
 
#endif /* defined(PID_CONTROL) */