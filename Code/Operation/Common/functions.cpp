#include <math.h>

#ifndef PI
#define PI 3.141592653589793115997963468544
#endif

void _SPEEDUPS(float* SPEEDUP_PI_k,
               float* SPEEDUP_PI_k_N,
               int    N,
               int    K) {
    for (int k=0; k<K; k++) { SPEEDUP_PI_k[k] = PI * float(k);
                              SPEEDUP_PI_k_N[k] = SPEEDUP_PI_k[k] / (N-1); } }

void _dfdx__2(float* dfdx__2,
              float* x_n__3,
              float* u_n__2) {
    float temp = (u_n__2[0] + u_n__2[1]) * 0.5;
    dfdx__2[0] = -sinf(x_n__3[2]) * temp;
    dfdx__2[1] =  cosf(x_n__3[2]) * temp; }

void _dfdu__6(float* dfdu__6,
              float* x_n__3,
              float* u_n__2,
              float  s_n ) {
    dfdu__6[0] = cosf(x_n__3[2]) * 0.5;
    dfdu__6[1] = dfdu__6[0];
    dfdu__6[2] = sinf(x_n__3[2]) * 0.5;
    dfdu__6[3] = dfdu__6[2];
    dfdu__6[4] = -1/s_n;
    dfdu__6[5] = -dfdu__6[4]; }

void _hk_inv__K_K(float* hk_inv__K_K,
                  int    N,
                  int    K,
                  float  dxdy,
                  float* SPEEDUP_PI_k_N) {
    float hk_single;
    float cos_i;
    float cos_j;
    float* temp_cos = new float[K*(N-1)];
    for (int k=0; k<K; k++) {
        for (int n=0; n<N-1; n++) {
            // temp_cos[k + n*K] = cosf(SPEEDUP_PI_k_N[k]*float(n));
            // temp_cos[k + n*K] = temp_cos[k+n*K] * temp_cos[k+n*K]; } }
            temp_cos[n + k*(N-1)] = cosf(SPEEDUP_PI_k_N[k]*float(n));
            temp_cos[n + k*(N-1)] = temp_cos[n + k*(N-1)] * temp_cos[n + k*(N-1)]; } }
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
            hk_single = 0.0;
            for (int ni=0; ni<N-1; ni++) {
                for (int nj=0; nj<N-1; nj++) {
                    cos_i = temp_cos[ni + ki*(N-1)];
                    cos_j = temp_cos[nj + kj*(N-1)];
                    hk_single += cos_i * cos_j; } }
            hk_inv__K_K[ki+kj*K] = 1 / powf(hk_single * dxdy, 0.5); } } 
    delete[] temp_cos; }

void _Fk__K_K(float* Fk__K_K,
              float* x_n__3,
              int    K,
              float* hk_inv__K_K,
              float* SPEEDUP_PI_k) {
    float* temp_i = new float[K];
    float* temp_j = new float[K];
    for (int k=0; k<K; k++) { temp_i[k] = cosf(SPEEDUP_PI_k[k] * x_n__3[0]);
                              temp_j[k] = cosf(SPEEDUP_PI_k[k] * x_n__3[1]); }
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
           Fk__K_K[ki + kj*K] = hk_inv__K_K[ki + kj*K] * temp_i[ki] * temp_j[kj]; } } 
    delete[] temp_i;
    delete[] temp_j; }

void _dFk__K_K_2(float* dFk__K_K_2,
                 float* x_n__3,
                 int    K,
                 float* hk_inv__K_K,
                 float* SPEEDUP_PI_k) {
    float  temp_i;
    float  temp_j;
    float* temp_sin_i = new float[K];
    float* temp_sin_j = new float[K];
    float* temp_cos_i = new float[K];
    float* temp_cos_j = new float[K];
    for (int k=0; k<K; k++) { temp_i = SPEEDUP_PI_k[k] * x_n__3[0];
                              temp_j = SPEEDUP_PI_k[k] * x_n__3[1];
                              temp_sin_i[k] = SPEEDUP_PI_k[k] * sinf(temp_i); 
                              temp_sin_j[k] = SPEEDUP_PI_k[k] * sinf(temp_j); 
                              temp_cos_i[k] = cosf(temp_i);
                              temp_cos_j[k] = cosf(temp_j); }
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
            dFk__K_K_2[ki+kj*K+0*K*K] = -hk_inv__K_K[ki+kj*K] * temp_sin_i[ki] * temp_cos_j[kj]; 
            dFk__K_K_2[ki+kj*K+1*K*K] = -hk_inv__K_K[ki+kj*K] * temp_sin_j[kj] * temp_cos_i[ki]; } } 
    delete[] temp_sin_i;
    delete[] temp_sin_j;
    delete[] temp_cos_i;
    delete[] temp_cos_j; }

void _Lambdak__K_K(float* Lambdak__K_K,
                   int    K) {
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
            Lambdak__K_K[ki+kj*K] = pow((1 + ki*ki + kj*kj),-1.5); } } }

float _E(int    K,
         float* Lambdak__K_K,
         float* phik__K_K,
         float* ck__K_K) {
    float E = 0;
    float temp;
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
            temp = ck__K_K[ki + kj*K] - phik__K_K[ki + kj*K];
            E = E + Lambdak__K_K[ki + kj*K] * temp * temp; } } 
    return E; }

void _dE__3(float* dE__3,
            float* x_n__3,
            int    K,
            float* hk_inv__K_K,
            float* Lambdak__K_K,
            float* phik__K_K,
            float* ck__K_K,
            float* SPEEDUP_PI_k) {
    float* dFk__K_K_2 = new float[K*K*2]; _dFk__K_K_2(dFk__K_K_2, x_n__3, K, hk_inv__K_K, SPEEDUP_PI_k);
    dE__3[0] = 0.0; 
    dE__3[1] = 0.0;
    float temp;
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
            temp = Lambdak__K_K[ki+kj*K] * (ck__K_K[ki+kj*K] - phik__K_K[ki+kj*K]) * 2;
            for (int i=0; i<2; i++) { dE__3[i] += temp * dFk__K_K_2[ki + kj*K + i*K*K]; } } } 
    delete[] dFk__K_K_2; }

void _dB__3(float*  dB__3,
            float* x_n__3,
            int    N,
            char*  dB__N_N) {
    int i = int(float(N)*x_n__3[0]);
    if      (i < 0)   {i = 0;}
    else if (i > N-1) {i = N-1;}
    int j = int(float(N)*x_n__3[1]);
    if      (j < 0)   {j = 0;}
    else if (j > N-1) {j = N-1;}
    char code = dB__N_N[i + j*N];
    dB__3[0] = float(code % 10 - 2); 
    dB__3[1] = float(code / 10 - 2); }

void _dD__2(float* dD__2,
            float* u_n__2,
            float  u_n_lim) {
    for (int i=0; i<2; i++) {dD__2[i] = u_n__2[0] / u_n_lim; } }

void _dldx__3(float* dldx__3,
              float* x_n__3,
              int    N,
              int    K,
              float* hk_inv__K_K,
              float* Lambdak__K_K,
              float* phik__K_K,
              float* ck__K_K,
              char*  dB__N_N,
              float  E_coef,
              float  B_coef,
              float* SPEEDUP_PI_k) {
    float* dB__3 = new float[3]; _dB__3(dB__3, x_n__3, N, dB__N_N);
    float* dE__3 = new float[3]; _dE__3(dE__3, x_n__3, K, hk_inv__K_K, Lambdak__K_K, phik__K_K, ck__K_K, SPEEDUP_PI_k);
    for (int i=0; i<2; i++) { dldx__3[i] = B_coef * dB__3[i] + E_coef * dE__3[i]; }
    delete[] dB__3;
    delete[] dE__3; }

void _dldu__2(float* dldu__2,
              float* u_n__2,
              float  u_n_lim,   
              float  D_coef) {
    float* dD__2 = new float[2]; _dD__2(dD__2, u_n__2, u_n_lim);
    for (int i=0; i<2; i++) { dldu__2[i] = D_coef * dD__2[i]; }
    delete[] dD__2; }

void _dp__3(float* dp__3,
            float* x_n__3,
            float* u_n__2,
            float* p__3,
            int    N,
            int    K,
            float* hk_inv__K_K,
            float* Lambdak__K_K,
            float* phik__K_K,
            float* ck__K_K,
            char*  dB__N_N,
            float  E_coef,
            float  B_coef,
            float* SPEEDUP_PI_k) {
    float* dldx__3 = new float[3]; _dldx__3(dldx__3, x_n__3, N, K, hk_inv__K_K, Lambdak__K_K, phik__K_K, ck__K_K, dB__N_N, E_coef, B_coef, SPEEDUP_PI_k); 
    float* dfdx__2 = new float[2]; _dfdx__2(dfdx__2, x_n__3, u_n__2);
    dp__3[0] = -dldx__3[0];
    dp__3[1] = -dldx__3[1];
    dp__3[2] = -(dfdx__2[0]*p__3[0] + dfdx__2[1]*p__3[1]); 
    delete[] dldx__3;
    delete[] dfdx__2; }

void _p_dec__3(float* p__3,
               float* dp__3,
               float  dt) {
    for (int s=0; s<3; s++) {p__3[s] = p__3[s] - dp__3[s]*dt;} }

void _du__2(float* du__2,
            float* x_n__3,
            float* u_n__2,
            float* p__3,
            float  u_n_lim,
            float  D_coef,
            float  s_n) {
    float* dldu__2 = new float[2]; _dldu__2(dldu__2, u_n__2, u_n_lim, D_coef); 
    float* dfdu__6 = new float[6]; _dfdu__6(dfdu__6, x_n__3, u_n__2, s_n);
    du__2[0] = (dfdu__6[0]*p__3[0] + dfdu__6[2]*p__3[1] + dfdu__6[4]*p__3[2]) + dldu__2[0];
    du__2[1] = (dfdu__6[1]*p__3[0] + dfdu__6[3]*p__3[1] + dfdu__6[5]*p__3[2]) + dldu__2[1]; 
    delete[] dldu__2; 
    delete[] dfdu__6; }

void _ck_push__K_K(int    step,
                   int    K,
                   float* ck__K_K,
                   float* ck__horizon_K_K) {
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
            ck__horizon_K_K[step*K*K + ki+kj*K] = ck__K_K[ki + kj*K]; } } }

void _ck_pull__K_K(int    step,
                   int    K,
                   float* ck__K_K,
                   float* ck__horizon_K_K) {
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
            ck__K_K[ki + kj*K] = ck__horizon_K_K[step*K*K + ki + kj*K]; } } }

void _ck_inc__K_K(float* ck__K_K,
                  float* x_n__3,
                  float  t,
                  int    K,
                  float* hk_inv__K_K,
                  float  dt,
                  float* SPEEDUP_PI_k) {
    float* Fk__K_K = new float[K*K]; _Fk__K_K(Fk__K_K, x_n__3, K, hk_inv__K_K, SPEEDUP_PI_k);
    for (int ki=0; ki<K; ki++) {
        for (int kj=0; kj<K; kj++) {
            ck__K_K[ki + kj*K] = (t * ck__K_K[ki + kj*K] + dt * Fk__K_K[ki + kj*K]) / (t + dt); } } 
    delete[] Fk__K_K; }
        
void _x_push__3(int    step,
                float* x_n__3,
                float* x_n__horizon_3) {
    for (int s=0; s<3; s++) {x_n__horizon_3[3*step + s] = x_n__3[s]; } }

void _x_pull__3(int    step,
                float* x_n__3,
                float* x_n__horizon_3) {
    for (int s=0; s<3; s++) {x_n__3[s] = x_n__horizon_3[3*step + s]; } }

void _x_inc__3(float* x_n__3,
               float* u_n__2,
               float  s_n,
               float  dt) {
    float eps = 0.0001;
    float temp = dt * (u_n__2[0] + u_n__2[1]) * 0.5;
    x_n__3[0] = x_n__3[0] + cosf(x_n__3[2]) * temp;
    x_n__3[1] = x_n__3[1] + sinf(x_n__3[2]) * temp;
    x_n__3[2] = x_n__3[2] + dt * (-u_n__2[0]+u_n__2[1]) / s_n; 
    if      (x_n__3[0] < eps)     { x_n__3[0] = eps; }
    else if (x_n__3[0] > 1 - eps) { x_n__3[0] = 1 - eps; }
    if      (x_n__3[1] < eps)     { x_n__3[1] = eps; }
    else if (x_n__3[1] > 1 - eps) { x_n__3[1] = 1 - eps; } }

void _u_push__2(int    step,
                float* u_n__2,
                float* u_n__horizon_2) {
    for (int s=0; s<2; s++) {u_n__horizon_2[2*step + s] = u_n__2[s]; } }

void _u_pull__2(int    step,
                float* u_n__2,
                float* u_n__horizon_2) {
    for (int s=0; s<2; s++) {u_n__2[s] = u_n__horizon_2[2*step + s]; } }

void _u_dec__2(float* u_n__2,
               float* du_n__2,
               float  u_n_lim,
               float  stepsize) {
    for (int s=0; s<2; s++) { u_n__2[s] = u_n__2[s] - stepsize*du_n__2[s]; }
    if      (u_n__2[0] < -u_n_lim) { u_n__2[0] = -u_n_lim; }
    else if (u_n__2[0] >  u_n_lim) { u_n__2[0] =  u_n_lim; }
    if      (u_n__2[1] < -u_n_lim) { u_n__2[1] = -u_n_lim; }
    else if (u_n__2[1] >  u_n_lim) { u_n__2[1] =  u_n_lim; } }

float _forward(float* x_n__3,
               float* u_n__2,
               float  t_i,
               int    K,
               float* hk_inv__K_K,
               float* ck__K_K,
               float* ck__horizon_K_K,
               float* x_n__horizon_3,
               float* u_n__horizon_2,
               int    horizon,
               float  s_n,
               float  dt,
               float* SPEEDUP_PI_k) {
    float t = t_i;
    for (int step=0; step<horizon; step++) {
        _x_push__3(step, x_n__3, x_n__horizon_3);               
        _ck_push__K_K(step, K, ck__K_K, ck__horizon_K_K);         
        _u_pull__2(step, u_n__2, u_n__horizon_2);            
        _x_inc__3(x_n__3, u_n__2, s_n, dt); 
        _ck_inc__K_K(ck__K_K, x_n__3, t, K, hk_inv__K_K, dt, SPEEDUP_PI_k);  
        t += dt; }        
    return t;
}
    
void _backward(float  t_f,
               float* x_n__3,
               float* u_n__2,
               int    N,
               int    K,
               float* hk_inv__K_K,
               float* Lambdak__K_K,
               float* phik__K_K,
               float* ck__K_K,
               float  u_n_lim,
               char*  dB__N_N,
               float* ck__horizon_K_K,
               float* x_n__horizon_3,
               float* u_n__horizon_2,
               int    horizon,
               float  E_coef,
               float  B_coef,
               float  D_coef,
               float  s_n,
               float  dt,
               float  stepsize,
               float* SPEEDUP_PI_k) {
    float* du_n__2 =   new float[2];
    float* dp__3 = new float[3];
    float* p__3 =    new float[3];
    for (int s=0; s<3; s++) {p__3[s] = 0.0;}
    float t = t_f;
    for (int step=horizon-1; step>=0; step--) {
        _u_pull__2(step, u_n__2, u_n__horizon_2); 
        _dp__3(dp__3, x_n__3, u_n__2, p__3, N, K, hk_inv__K_K, Lambdak__K_K, phik__K_K, ck__K_K, dB__N_N, E_coef, B_coef, SPEEDUP_PI_k);  
        _p_dec__3(p__3, dp__3, dt);
        _du__2(du_n__2, x_n__3, u_n__2, p__3, u_n_lim, D_coef, s_n);          
        _u_dec__2(u_n__2, du_n__2, u_n_lim, stepsize); 
        t -= dt;                           
        _u_push__2(step, u_n__2, u_n__horizon_2);      
        _x_pull__3(step, x_n__3, x_n__horizon_3);       
        _ck_pull__K_K(step, K, ck__K_K, ck__horizon_K_K); }
    delete[] du_n__2;
    delete[] dp__3;
    delete[] p__3;

}

