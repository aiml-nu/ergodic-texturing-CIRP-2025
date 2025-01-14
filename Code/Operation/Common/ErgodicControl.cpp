#include <math.h>
#include <iostream>
#include <iomanip>
#include "ErgodicControl.hpp"
#include "functions.cpp"
#include "constants.hpp"

ErgodicControl::ErgodicControl() {
    // Initializing all variables
    x_n__3[0] = X_N_0;
    x_n__3[1] = Y_N_0;
    x_n__3[2] = TH_N_0;
    for (int s=0; s<2;           s++) { u_n__2[s] =             U_N_INIT; }
    for (int s=0; s<K;           s++) { SPEEDUP_PI_k[s] =       0.0;
                                        SPEEDUP_PI_k_N[s] =     0.0; }
    for (int s=0; s<K*K;         s++) { hk_inv__K_K[s] =        0.0;  
                                        Fk__K_K[s] =            0.0; 
                                        ck__K_K[s] =            0.0; 
                                        Lambdak__K_K[s] =       0.0; }
    for (int s=0; s<2*HORIZON;   s++) { u_n__horizon_2[s] =     U_N_INIT; }  
    for (int s=0; s<HORIZON;     s++) { x_n__horizon_3[0+s*3] = X_N_0;
                                        x_n__horizon_3[1+s*3] = Y_N_0;
                                        x_n__horizon_3[2+s*3] = TH_N_0; }
    for (int s=0; s<HORIZON*K*K; s++) { ck__horizon_K_K[s] =    0.0; }
    t = 0;
    dxdy = (1)/(float(N-1)*float(N-1)); }

ErgodicControl::~ErgodicControl() { }

void ErgodicControl::Startup() {
    _SPEEDUPS(SPEEDUP_PI_k, SPEEDUP_PI_k_N, N, K);
    _hk_inv__K_K(hk_inv__K_K, N, K, dxdy, SPEEDUP_PI_k_N); 
    _Lambdak__K_K(Lambdak__K_K, K);
}

void ErgodicControl::UpdateControlHorizon(float new_x, float new_y, float new_theta, float new_t) {
    t = new_t;
    x_n__3[0] = new_x;
    x_n__3[1] = new_y;
    x_n__3[2] = new_theta;
    float t_f = _forward(x_n__3, u_n__2, t, K, hk_inv__K_K, ck__K_K, ck__horizon_K_K, x_n__horizon_3, u_n__horizon_2, HORIZON, S_N, MPC_DT, SPEEDUP_PI_k);
    _backward(t_f, x_n__3, u_n__2, N, K, hk_inv__K_K, Lambdak__K_K, phik__K_K, ck__K_K, U_N_LIM, dB__N_N, ck__horizon_K_K, x_n__horizon_3, u_n__horizon_2, HORIZON, E_COEF, B_COEF, D_COEF, S_N, MPC_DT, STEPSIZE, SPEEDUP_PI_k); }

void ErgodicControl::UpdateResultDistribution(float new_x, float new_y, float new_theta, float new_t) {
    t = new_t;
    x_n__3[0] = new_x;
    x_n__3[1] = new_y;
    x_n__3[2] = new_theta;
    _ck_inc__K_K(ck__K_K, x_n__3, t, K, hk_inv__K_K, SIM_DT, SPEEDUP_PI_k); }

void ErgodicControl::Reset() {
    for (int s=0; s<K*K;         s++) {ck__K_K[s] =         0.0;} 
    for (int s=0; s<HORIZON*K*K; s++) {ck__horizon_K_K[s] = 0.0;} 
    for (int s=0; s<2*HORIZON;   s++) {u_n__horizon_2[s] =  U_N_INIT;}
    for (int s=0; s<3*HORIZON;   s++) {x_n__horizon_3[s] =  0.0;}
    t = 0; 
}

float ErgodicControl::GetControlLeft(int step) {
    return u_n__horizon_2[2*step + 0]; }

float ErgodicControl::GetControlRight(int step) {
    return u_n__horizon_2[2*step + 1]; }

float ErgodicControl::GetErgodicMetric() {
    return _E(K, Lambdak__K_K, phik__K_K, ck__K_K); }