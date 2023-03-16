#include <iostream>
#include <fstream>

#include "boost/thread.hpp"

using namespace std;

class CONTROLLER {
    public:
        CONTROLLER(double Kp, double Ki, double Kd, double xdes, double dt);
        
        void loop();               //Main loop function        
        void system_start();       //start the system
        void set_xdes(double x);   //member to set the desired value
        double simulate_system(const double A,const double B, double state, double u); //simula il processo da controllare

    private:
        double _Kp;
        double _Ki;
        double _Kd;
        double _xdes;
        double _state;
        double _u;
        double _dt;
};
