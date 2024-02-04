#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "FastPWM.h"
#define PI 3.14159265358979323846
#define PWMfrequency 5000
#define PWMperiodTicks 12000 // equals to (60 000 000 / PWMfrequency), and we choose to use 5kHz PWM

/* Implementing a workaround to bypass an ISR-related error by substituting 
   the standard AnalogIn with our custom MyAnalogIn class throughout the experiment. 
   This change is crucial but the details are not essential for understanding the overall experiment setup.
   Please ignore the implementation but use MyAnalogIn everytime you want to use AnalogIn
*/
class MyAnalogIn : public AnalogIn {
public:
    MyAnalogIn(PinName inp) : AnalogIn(inp) { }
    virtual void lock() { }
    virtual void unlock() { }
};
/* --------------------------------------------------- */

// Define number of communication parameters with matlab
#define NUM_INPUTS 2
#define NUM_OUTPUTS 5


Serial pc(USBTX, USBRX,115200);     // USB Serial Terminal for debugging
ExperimentServer server;            // Object that lets us communicate with MATLAB
Timer t;                            // Timer to measure elapsed time of experiment
DigitalOut led_g(LED_GREEN,1);      // UDP server state indicator

/* ===== Complete the code in this block =====
 * Assign appropriate digital/analog pin number for control and sensing in the 
 * Example: For digital pin 1, you have to enter D1
 * Example: For analog  pin 1, you have to enter A1
*/
FastPWM    M1PWM(D*);               // Motor PWM output (we are using the "FastPWM" rather than built-in "PwmOut" for higher resolution)
DigitalOut M1INA(D*);               // Motor forward enable
DigitalOut M1INB(D*);               // Motor backward enable
**********    CS(**);               // Current sensor
/* ===== End of code block =================== */

// Create a quadrature encoder
// 64(counts/motor rev)*18.75(gear ratio) = 1200(counts/rev)
// Pins A, B, no index, 1200 counts/rev, Quadrature encoding
// Note: Reversed A & B to match motor direction
QEI encoder(D5,D3, NC, 1200 , QEI::X4_ENCODING); 
const float radPerTick = 2.0*PI/1200.0;

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM);

const float SupplyVoltage = 12;     // Supply voltage in Volts
// Set motor voltage in (V)
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM);


/* Main function that would run on the FRDM board
 * Note: unlike Arduino, we do not have a setup function and a loop function
 * The main function would only run once. To have a loop, we have to use while loop by ourselves
*/ 
int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    led_g = 0;  // UDP server is ready, turn on green led

    // Setting PWM frequency. Here is 5k Hz, same as the current loop
    M1PWM.prescaler(1);
    M1PWM.period_ticks(PWMperiodTicks);
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    // infinite while loop, analogous to Arduino's loop function
    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) { // Check whether we have parameter
            // Unpack parameters from MATLAB
            float voltage = input_params[0]; // Applied voltage
            float ExpTime = input_params[1]; // Expriement time in second
        
            // Setup experiment
            t.reset(); // Reset timer
            t.start(); // Start timer so that we have elapsed time of experiment
            encoder.reset();
            setMotorVoltage(0,M1INA,M1INB,M1PWM); //Turn off motor just in case

            // Run experiment until timeout
            while( t.read() < ExpTime ) {
                // Apply the commanded voltage to motor
                setMotorVoltage(voltage,M1INA,M1INB,M1PWM);

                /* ===== Complete the code in this block =====
                 * Fix the current sensing code below based on your result from pre-lab quiz
                 * Note: CS gives the analog reading in range [0.0, 1.0] as a float variable
                */
                float current  = 0.0f * CS; // Read the current sensor value
                /* ===== End of code block =================== */

                // Read angle from encoder
                float angle    = (float)encoder.getPulses()   * radPerTick; // in rad
                float velocity =        encoder.getVelocity() * radPerTick; // in rad/s
                
                // Fill the output data to send back to MATLAB
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = angle;
                output_data[2] = velocity;
                output_data[3] = voltage;
                output_data[4] = current;
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                wait(.001); // Running this loop and sending data in roughly 1kHz
            } // end of experiment loop

            // Cleanup and turn off motor after experiment
            server.setExperimentComplete();
            setMotorVoltage(0,M1INA,M1INB,M1PWM);
        } // end if of "check whether we have parameter"
    } // end while of "infinite while loop"
} // end main

//Set motor voltage (nagetive means reverse)
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM){
    setMotorDuty(voltage / SupplyVoltage, INA, INB, PWM);
}

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM)
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

    // The standard way to set built-in PWM is: 
    // PWM.write(duty);
    // However, we are using a high resolution PWM (FastPWM library). 
    // We have to set the PWM is the following way.
    PWM.pulsewidth_ticks((int) (PWMperiodTicks*duty));
}