#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#define PI 3.14159265358979323846

// Define number of communication parameters with matlab
#define NUM_INPUTS 2
#define NUM_OUTPUTS 3


Serial pc(USBTX, USBRX,115200);     // USB Serial Terminal for debugging
ExperimentServer server;            // Object that lets us communicate with MATLAB
Timer t;                            // Timer to measure elapsed time of experiment

/************************Complete the code in this block**************************/
// Assign digital/analog pins for control and sensing
PwmOut     M1PWM(D9);              // Motor PWM output
DigitalOut M1INA(D2);               // Motor forward enable
DigitalOut M1INB(D4);               // Motor backward enable
AnalogIn   CS(A2);                  // Current sensor
/*********************************************************************************/

// Create a quadrature encoder
// 64(counts/motor rev)*18.75(gear ratio) = 1200(counts/rev)
// Pins A, B, no index, 1200 counts/rev, Quadrature encoding
QEI encoder(D3,D5, NC, 1200 , QEI::X4_ENCODING); 
const float radPerTick = -2.0*PI/1200.0;

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM);

const float SupplyVoltage = 12;     // Supply voltage in Volts
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM);

int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();

    // PWM period should nominally be a multiple of our control loop
    M1PWM.period_us(50);
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) {
            // Unpack parameters from MATLAB
            float voltage = input_params[0]; // Applied voltage
            float ExpTime = input_params[1]; // Expriement time in second
        
            // Setup experiment
            t.reset();
            t.start();
            encoder.reset();
            setMotorVoltage(0,M1INA,M1INB,M1PWM); //Turn off motor just in case

            // Run experiment until timeout
            while( t.read() < ExpTime ) { 
                setMotorVoltage(voltage,M1INA,M1INB,M1PWM);

/************************Complete the computation of current sensing***************/
                // Note: CS gives the analog reading in range [0.0, 1.0]
                // Your result should obtain from the logic voltage and datasheet of the sensor
                // Read the current sensor value
                float current = 36.7f * CS - 18.4f;
/*********************************************************************************/
                // Read angle from encoder
                float angle = (float)encoder.getPulses()*radPerTick;
                
                // Form output to send to MATLAB    
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();  // timestamp
                output_data[1] = current;
                output_data[2] = angle;
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                wait(.001);                 // Sending data in 1kHz
            }     
            // Cleanup and turn off motor after experiment
            server.setExperimentComplete();
            setMotorVoltage(0,M1INA,M1INB,M1PWM);
        } // end if
    } // end while
} // end main

//Set motor voltage (nagetive means reverse)
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM){
    setMotorDuty(voltage / SupplyVoltage, INA, INB, PWM);
}

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM)
{
    unsigned char reverse = 0;

    if (duty < 0) {
        duty = -duty;  // Make duty a positive quantity
        reverse = 1;  // Preserve the direction
    }

    if (duty == 0) {
        INA = 0;  // Make the motor coast no
        INB = 0;  // matter which direction it is spinning.
    } else if (reverse) {
        INA = 0;
        INB = 1;
    } else {
        INA = 1;
        INB = 0;
    }

    PWM.write(duty);
}