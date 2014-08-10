//
//  PID.cpp
//  PID
//
//  Created by Yosub Lee & Peter Quan on 7/15/14.
//  Copyright (c) 2014 Yosub Lee & Peter Quan. All rights reserved.
//

/**
PID control code that enables the user to set the desired set point.
Based on open source code found here: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
*/
 
#include "PID.h"
 
PID::PID()
{
    initialized = 0;
    setPoint(0);
    m_input     = 0;
    m_output    = 0;
    m_setpoint  = 0;
    m_iTerm    = 0;
    m_lastInput = 0;
    m_sampleTime = 0;   
    m_sampleTimeSec = 0;
    m_ratio = 0;
    m_outMin = 0;
    m_outMax = 0;
}
 
PID::~PID(){}

/*
Specifies the set point of the PID algorithm
@param point
*/ 
void PID::setPoint(double point)
{
    m_setpoint = point;
}

/*
Specifies the gain coefficients
@param kp, ki, kd
*/ 
void PID::setGains(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    std::cout << "P: " << this->kp << " I: " << this->ki << " D:" << this->kd << std::endl;
    std::cout << "Initialized \n";
    initialized = true;
}

/*
Adjust gain coefficients in real time
@param kp2, ki2, kd2
*/ 
void PID::setTunings(double kp2, double ki2, double kd2){
    m_sampleTimeSec = m_sampleTime / 1000;
    kp = kp2;
    ki = ki2 * m_sampleTimeSec;
    kd = kd2 / m_sampleTimeSec;
}

/*
-Sets new sample time
-Adjusts gain coefficients based on the ratio of new and previous
 sample times
 @param newSampleTime
*/ 
void PID::setSampleTime(double newSampleTime){
    if(newSampleTime > 0){
        m_ratio = newSampleTime / m_sampleTime;
        ki *= m_ratio;
        kd /= m_ratio;
        m_sampleTime = newSampleTime;
    }
}

void PID::setOutputLimits(double min, double max){
    if(min > max) 
        return;

    m_outMin = min;
    m_outMax = max;

    if(m_output > m_outMax)
        m_output = m_outMax;

    else if(m_output < m_outMin)
        m_output = m_outMin;

    if(m_iTerm > m_outMax)
        m_iTerm = m_outMax;

    else if(m_iTerm < m_outMin)
        m_iTerm = m_outMin;
}
 
/*
-Derivative Kick is accounted for, thus enabling the user to change
 the set point in real time
-Has the ability to tune parameters while the system is running 
 without undersiable bumps occuring 
 @param input 
 @return PID output
*/
double PID::compute(double input)
{
    if ( initialized ) {
        m_input = input;

        // Sample Time
        m_now = chrono::high_resolution_clock::now();

        float timeDiff=(chrono::duration_cast<chrono::microseconds>(m_now-m_lastTime).count())/100000.0;
        //cout << "What Jame's clock got: " << timeDiff << endl;
        if(timeDiff >= m_sampleTime){
            // Compute all the working error variables
            double error = m_setpoint - m_input;
            m_iTerm += (ki * error);
            if(m_iTerm > m_outMax)
                m_iTerm = m_outMax;
            else if(m_iTerm < m_outMin)
                m_iTerm = m_outMin;
            double dInput = (m_input - m_lastInput);
            // Compute PID Output
            m_output = kp * error + ki * m_iTerm + kd * dInput;
            if(m_output > m_outMax)
                m_output = m_outMax;
            else if(m_output < m_outMin)
                m_output = m_outMin; 
            // Remember some variables for next time
            m_lastInput = m_input;
            m_lastTime = m_now;
        }
         
        return m_output;
    }
    else {
        return 0;
    }
}