#include "Neuro_PID.h"

Neuro_PID::Neuro_PID() {
    initWeights();
}

void Neuro_PID::initWeights() {
        // Layer 1 Weights (Hidden Layer)
    W1[0][0] = -0.518939f; W1[0][1] = 0.013609f; W1[0][2] = 0.628119f;
    W1[1][0] = 0.012907f; W1[1][1] = 0.911654f; W1[1][2] = -0.799642f;
    W1[2][0] = 0.263670f; W1[2][1] = 0.467880f; W1[2][2] = 0.315470f;
    W1[3][0] = -0.368414f; W1[3][1] = 1.015138f; W1[3][2] = 0.005965f;
    W1[4][0] = 0.279971f; W1[4][1] = 0.115497f; W1[4][2] = 0.845667f;

    B1[0] = 0.026791f;
    B1[1] = 0.119306f;
    B1[2] = 0.263766f;
    B1[3] = -0.313573f;
    B1[4] = 0.544818f;

    W2[0][0] = 0.253295f; W2[0][1] = -0.466446f; W2[0][2] = 0.142002f; W2[0][3] = -0.638513f; W2[0][4] = -0.509502f;
    W2[1][0] = 0.965656f; W2[1][1] = -0.324353f; W2[1][2] = 0.088890f; W2[1][3] = -0.830743f; W2[1][4] = 0.212690f;
    W2[2][0] = -0.130944f; W2[2][1] = 0.200678f; W2[2][2] = -0.296468f; W2[2][3] = 0.362294f; W2[2][4] = -0.637993f;

    B2[0] = -0.200781f;
    B2[1] = 0.524034f;
    B2[2] = 0.001309f;

}

// Neuro_PID.cpp: single-precision activation functions

float Neuro_PID::sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

float Neuro_PID::tanh_act(float x) {
    return tanhf(x);
}

// Neuro_PID.cpp: single-precision forward pass

void Neuro_PID::computePID(
    float error,
    float d_error,
    float i_error,
    float& kp,
    float& ki,
    float& kd
) {
    const float inputs[NUM_INPUTS] = {
        error,
        d_error,
        i_error
    };

    float hidden[NUM_HIDDEN] = {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    };

    float outputs[NUM_OUTPUTS] = {
        0.0f,
        0.0f,
        0.0f
    };

    for (int i = 0; i < NUM_HIDDEN; ++i) {
        float sum = B1[i];

        for (int j = 0; j < NUM_INPUTS; ++j) {
            sum += W1[i][j] * inputs[j];
        }

        hidden[i] = tanhf(sum);
    }

    for (int i = 0; i < NUM_OUTPUTS; ++i) {
        float sum = B2[i];

        for (int j = 0; j < NUM_HIDDEN; ++j) {
            sum += W2[i][j] * hidden[j];
        }

        outputs[i] = 1.0f / (1.0f + expf(-sum));
    }

    kp = outputs[0] * 8.0f;
    ki = outputs[1] * 2.0f;
    kd = outputs[2] * 0.2f;
}