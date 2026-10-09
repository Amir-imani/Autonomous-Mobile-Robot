#ifndef NEURO_PID_H
#define NEURO_PID_H

#include <Arduino.h>
#include <math.h>

class Neuro_PID {
private:
    // ابعاد شبکه عصبی
    static const int NUM_INPUTS = 3;   
    static const int NUM_HIDDEN = 5;   
    static const int NUM_OUTPUTS = 3; 

    float W1[NUM_HIDDEN][NUM_INPUTS];
    float B1[NUM_HIDDEN];
    float W2[NUM_OUTPUTS][NUM_HIDDEN];
    float B2[NUM_OUTPUTS];

    float sigmoid(float x);
    float tanh_act(float x);

public:
    Neuro_PID();
    
    void initWeights();

    void computePID(float error, float d_error, float i_error, float &kp, float &ki, float &kd);
};

#endif