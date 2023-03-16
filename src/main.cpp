#include "control_loop.h"

using namespace std;

//the goal is to implement a simple PID controller, 
//running on input value to reach the desired one

// Sense: read a value from keyboard
// Plan:  generate the correct input
// Act:   set the input

double dt;
double Kp;
double Ki;
double Kd;
double xdes;

int main(int argc, char** argv) {
    Kp = atof(argv[1]);
    Ki = atof(argv[2]);
    Kd = atof(argv[3]);
    dt = atof(argv[4]);
    cout<<"Inserire il riferimento iniziale"<<endl;
    cin >> xdes;

   

    CONTROLLER contr(Kp,Ki,Kd,xdes,dt);

    while(true){
    cout<<"Inserire il riferimento iniziale"<<endl;
    cin >> xdes;
    contr.set_xdes(xdes);
    }


    return 0;
}
