#include <chrono>
#include <ctime>
#include <stdio.h>

#include "constants.hpp"
#include "ErgodicControl.cpp"

/*
   ~~~ IMPORTANT README ~~~

Make sure not to run using "Code Runner" or other tools from VS Code. Just compile and run, i.e. in terminal:

(cd to this directory first, then...)
g++ .\demo.cpp -o demo
./demo

Running otherwise generated some weird bugs...
*/

#define DEBUG // Causes printouts to std::cout

// Files for output storage
FILE* ck_file =   fopen("Demo_Outputs/ck.out","wb");
FILE* x_file =    fopen("Demo_Outputs/x.out","wb");
FILE* u_file =    fopen("Demo_Outputs/u.out","wb");
FILE* E_file =    fopen("Demo_Outputs/E.out","wb");

float x = X_N_0; 
float y = Y_N_0;
float theta = TH_N_0;
float x_n__3[3];
float v_left = 0.0;
float v_right = 0.0;
float u_n__2[2];
float t = 0;
float metric;

int main() {
    ErgodicControl controller;
    controller.Startup();

    auto start = std::chrono::system_clock::now();
    std::time_t start_time = std::chrono::system_clock::to_time_t(start);

    float eps = 0.0001;
    for (int i=0; i<TIME_STEPS; i++) {
        controller.UpdateControlHorizon(x,y,theta,t);
        v_left = controller.GetControlLeft(0);
        v_right = controller.GetControlRight(0);
        x = x + SIM_DT * cosf(theta) * (v_left + v_right) * 0.5;
        if      (x < eps)     { x = eps; }
        else if (x > 1 - eps) { x = 1 - eps; }
        y = y + SIM_DT * sinf(theta) * (v_left + v_right) * 0.5;
        if      (y < eps)     { y = eps; }
        else if (y > 1 - eps) { y = 1 - eps; }
        theta = theta + SIM_DT * (-v_left + v_right) / S_N;
        t = t + SIM_DT; 
        controller.UpdateResultDistribution(x,y,theta,t);
        if (i % SAVE_EVERY == 0) {
            x_n__3[0] = x;
            x_n__3[1] = y;
            x_n__3[2] = theta;
            u_n__2[0] = v_left;
            u_n__2[1] = v_right;
            metric = controller.GetErgodicMetric();
            fwrite(x_n__3,  sizeof(float), sizeof(x_n__3)/sizeof(float), x_file); 
            fwrite(u_n__2,  sizeof(float), sizeof(u_n__2)/sizeof(float), u_file); 
            fwrite(&metric, sizeof(float), 1, E_file); 
            #ifdef DEBUG
            std::cout << "x: "
                      << x 
                      << " y: " 
                      << y 
                      << " theta: " 
                      << theta 
                      << " t: " 
                      << t
                      << " v_left: "
                      << v_left
                      << " v_right: "
                      << v_right
                      << std::endl;
            #endif
        }
    }

    auto end = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::chrono::duration<double> elapsed_seconds = end-start;

    fwrite(controller.ck__K_K, sizeof(float), sizeof(controller.ck__K_K)/sizeof(float), ck_file); 

    std::cout << "Total time for "
              << TIME_STEPS 
              << " iterations was " 
              << elapsed_seconds.count() 
              << "s, giving a frequency of " 
              << TIME_STEPS/elapsed_seconds.count() 
              << " Hz" 
              << std::endl; 
    
    fclose(ck_file);   free(ck_file);
    fclose(x_file);    free(x_file);
    fclose(u_file);    free(u_file);
    fclose(E_file);    free(E_file);

    return 0;
}