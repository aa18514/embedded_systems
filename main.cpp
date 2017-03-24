#include "mbed.h"
#include "rtos.h"
#include "slre.h"

 //Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motofr Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
Serial pc(SERIAL_TX, SERIAL_RX);
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = -2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

//Quadrature Outputs
DigitalIn Q1(CHA);
DigitalIn Q2(CHB);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);


//------------------------------------------------------------------------//

//Thread definitions
Thread thread1;

//Interrupt definitions
InterruptIn I3set(I3pin);
//InterruptIn time_i(I2pin);


int cRev = 0; //revolution counter
double defined_velocity = 18;
double cVelocity = 0; //setting the cVelocity
double oldVelocity = 0;
int s = 0; //distance to target

Timer timer;
double time1Rev = 0; //time in microseconds.
double cTime = 0;
int nRevs = 0;


double controller(){
    int kr = 5;
    double kpr = 10;
    double kdr = -35;
    double kir = nRevs/100;
    double kpv = 2;
    double kdv = 0;
    double kv = 5;
    //double pwm;
    //double delta,pwm_velocity,pwm_distance;   //converting to seconds
    
    s = nRevs - cRev;
    double pwmR = kr*((kpr*s) + (kdr*cVelocity+kir) + 1);
    
    if( pwmR > 1)
        pwmR = 1;
    else if (pwmR < 0.1)
        pwmR = 0.1;
        
    
    double v = defined_velocity - cVelocity;
    double a = (cVelocity - oldVelocity) / time1Rev;
    
    double pwmV = kv*((kpv*v) + (kdv*a));
        if( pwmV > 1)
        pwmV = 1;
    else if (pwmV < 0.1)
        pwmV = 0.1;
    
    if (pwmR < pwmV)
        return pwmR;
    else
        return pwmV;   
}

//Set a given drive state
void motorOut(int8_t driveState) {
    
    //pwmval = 1;
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    
    double pwmval;
    
    
    pwmval = controller();
    //printf("PWM duty cycle: %f\n\r",pwmval);
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    
    if (driveOut & 0x01) L1L = pwmval;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = pwmval;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = pwmval;
    if (driveOut & 0x20) L3H = 0;
}

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState() {
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    cRev = 0;

    //Get the rotor state
    return readRotorState();
}

void runMotor() {
    int8_t orState = 0;    //Rotot offset at motor state 0

    //Initialise the serial port (and the timer)
    RawSerial pc(SERIAL_TX, SERIAL_RX);
    int8_t intState = 0;
    int8_t intStateOld = 0;

    //mutex1.lock();
    //pc.printf("Helloo\n\r");
    //mutex1.unlock();

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (cRev <= nRevs) {
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
        //int old_revc = -1;
        //if(old_revc != cRev){
        //printf("cRev: %d\n", cRev);
        //    old_revc = cRev;
    
    }
}

void rev_c_fn(){
    float time = timer.read();
    time1Rev = time - cTime;
    cTime = time;
    oldVelocity = cVelocity;
    cVelocity = 1/(time1Rev);
    cRev++;
}
bool spin_Motor = false;
struct cap captures[5 + 1];
void recieve_instruction(){
    char result[20];
    int n = pc.scanf("%s", result); 
    struct slre slre, slre2;
    slre_compile(&slre, "R(\d*)"); 
    slre_compile(&slre2, "R-(\d*)"); 
    if(slre_match(&slre, result, 3, captures)){
        spin_Motor = true; 
        lead = 2;
    }
    else if(slre_match(&slre2, result, 3, captures)){
        spin_Motor = true; 
        lead = -2; 
    }
}
//Main--------------------------------------------------------------------------
int main() {
    pc.baud(9600);
    //pc.attach(&recieve_instruction);
    while(1){
        recieve_instruction(); 
        if(spin_Motor){
            spin_Motor = false; 
            nRevs = atoi(captures[1].ptr); 
            timer.start();
            thread1.start(runMotor);
            //thread2.start(control_torque);
            I3set.rise(&rev_c_fn);
            /*
            int old_revc = -1;
            while(1) {
                if(old_revc != cRev){
                    printf("cRev: %d\n", cRev);
                    //revolutions per second
                    //printf("time_1_rev: %d\n", time1Rev);
                    //printf("%f\n", cVelocity);
                    old_revc = cRev;
                }
            }*/
        }
    }
}
