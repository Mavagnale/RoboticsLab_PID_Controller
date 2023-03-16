#include "control_loop.h"

//We can use the class constructor to set parameters
CONTROLLER::CONTROLLER(double Kp, double Ki, double Kd, double xdes, double dt) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    set_xdes(xdes);    
    _dt = dt;

    boost::thread loop_thread(&CONTROLLER::loop,this);
}


//Sense: get input to change the state of our System
void CONTROLLER::set_xdes(double xdes) {
    _xdes = xdes;
}


//Random initial value
void CONTROLLER::system_start() {
    srand((unsigned)time(NULL));
    _state = rand() / (RAND_MAX + 1.0);
}

double CONTROLLER::simulate_system(const double A,const double B, double state, double u){      //sistema dinamico del primo ordine
    return A*state + B*u;
}

void CONTROLLER::loop() {
    const double A = 0.5;  //parametri del sistema dinamico
    const double B = 1;    

    double e = 0, p = 0, i = 0, d = 0;
   // system_start();
    _state = 0;
    _u = 0;
    int k = 0;          //passo di simulazione
    double eold = 0;    //errore al passo precedente
    double xmax = 0;

    // "svuota" i file di testo
    ofstream f_state("stato.txt");
    ofstream f_u("ingresso.txt");
    f_state << "";
    f_u << "";
    f_state.close();
    f_u.close();

    cout <<"Stato iniziale: " <<_state << endl; 
    cout <<"Ingresso iniziale: " <<_u << endl; 

    while (abs(e - eold)>0.0001 || k==0) { //simula fino a che non va a regime

        ofstream f_state("stato.txt",std::ios_base::app);
        ofstream f_u("ingresso.txt",std::ios_base::app);
        f_state<< _state <<endl;
        f_u<< _u <<endl;
        f_state.close();
        f_u.close();

        eold = e;
        e = _xdes - _state;
        p = e;
        i = i + e*_dt;
        d = (e - eold)/_dt;
        // DEBUG //
        /*
        cout<<"Kp*e="<<_Kp*e<<endl; 
        cout<<"Ki*i="<<_Ki*i<<endl;
        cout<<"Kd*d="<<_Kd*d<<endl;
        cout<<"t = "<<k*_dt<<endl;
        */
        _u =  _Kp*p + _Ki*i + _Kd*d;
    
        _state = simulate_system(A,B,_state,_u);
        if (_state > xmax) xmax = _state;

        cout <<"Stato : " <<_state << endl;       
        cout <<"Ingresso : " <<_u << endl; 
        cout <<"t = " <<k*_dt << endl; 

        usleep(_dt*1000000);
        k++;
    }



    cout << "Regime raggiunto in " << k*_dt << "s" << endl;
    cout << "Sovraelongazione massima: " << (xmax -_xdes)/_xdes*100 << "%" << endl; 
}


