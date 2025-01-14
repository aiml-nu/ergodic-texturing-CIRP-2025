#ifndef ERGODICCONTROL_H
#define ERGODICCONTROL_H

#include "constants.hpp"

#ifndef PI
#define PI 3.141592653589793115997963468544
#endif

class ErgodicControl {
    public:
        // State
        float x_n__3[3];                 // Robot state [x,y,theta] normalized by the domain size
        float x_n__horizon_3[HORIZON*3]; // States over the MPC horizon

        // Control
        float u_n__2[2];                 // Robot controls [v_left,v_right] normalized by the domain size
        float u_n__horizon_2[2*HORIZON]; // Controls over the MPC horizon
        
        // Input distribution and spatial Fourier coefficients 
        float hk_inv__K_K[K*K];             // Precalculated inverse of hk
        float Fk__K_K[K*K];                 // Basis functions evaluated at one (x,y) pair
        float ck__K_K[K*K];                 // Trajectory spatial statistics
        float ck__horizon_K_K[HORIZON*K*K]; // Trajectory spatial statistics over the horizon
        float Lambdak__K_K[K*K];            // Normalizing term
        float dxdy;                         // Spatial numerical integration size

        // Speedups
        float SPEEDUP_PI_k[K];
        float SPEEDUP_PI_k_N[K];
        
        // Time
        float t; // Current time

        ErgodicControl();
        ~ErgodicControl();

        void Startup();
        void UpdateControlHorizon(float new_x, float new_y, float new_theta, float new_t); 
        void UpdateResultDistribution(float new_x, float new_y, float new_theta, float new_t);
        void Reset(); 

        float GetControlLeft(int step);
        float GetControlRight(int step); 
        float GetErgodicMetric();
};

#endif
