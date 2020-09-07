#include "mbed.h"
#include "QEI.h"


//Pins
DigitalOut in1(PD_3);           //Pin 13
DigitalOut in2(PB_14);          //Pin 12
PwmOut pwmPin(PB_15);           //Pin 11
                                

//Functions
void initMotor(void);
void turnCW(void);
void turnCCW(void);
void stopMotor(void);
void setSpeed(float input);

//Entities
Serial serial(USBTX, USBRX);
QEI encoder (PG_10, PH_6, NC, 2, QEI::X2_ENCODING); //Pin 10 and 8 are used for encoder 
Timer t;

//Constants
const int stabilizationPeriodAllowance = 7;
const int stopTimeTreshold = 3;
const float degreePerPulse = 2.41;         //2.41  
const float testDegree = 1080.0;      
const float integralVicinityMax = 20;
const float integralVicinityMin = degreePerPulse * 2;
const float pwmFrictionLimit = 0.2;       //Amount of pwm necessary to keep the motor revolving just barely is 0.215, we will give 0.2 so that it requires kp a little
const float kpStartingValue = 0.08;
const float kpIncrement = 0.01;
const float overshootLimit = degreePerPulse * 2;
const float maximumTuningAttempts = 1;

//Variables
float kc;
float Tc;
float kp;     
float ki;         
float kd;
float kiDecrement;
float kdIncrement;

float newError;
float oldError;
float newTime;
float oldTime;
float derivative;
float integral;
float dt;
float overshoot;

bool increasing;
bool decreasing;
int index;
float timeData[200]; 
float degreesCovered;

bool stopped;


int main()
{    
    serial.printf("Hello!\n");      //Ensuring that the system has reset and that the serial communication is working correctly.
    initMotor();                    //Initialize the motor so that the correct frequency of PWM is set and there is not surge when the first PWM is supplied.
    
    kc = kpStartingValue;           //Start with the designated value.
    kc -= kpIncrement;              //Decrease one increment so that when the system first adds it, the first tried value is the determined starting value.
    t.reset();        
    
    do
    {
        //Resetting the parameters that require an update at each iteration of the loop.
        kc += kpIncrement;          
        index = 0;                  
        oldTime = 0;
        oldError = testDegree;      //Set to the test degree so that in the first iteration of the loop, there is no change between the errors, which would otherwise indicate turning.
        stopped = false;
        t.start();
        while(t.read() < stabilizationPeriodAllowance && !stopped)  //The system keeps on tring to achieve the target degree until it has stopped or ran out of time.
        {
            degreesCovered = encoder.getPulses() * degreePerPulse;  //Current degree from the starting position.
            newError = testDegree - degreesCovered;                 //Error is updated immediately.
            setSpeed( kc * newError + pwmFrictionLimit );           //Set speed function is used to supply the motor with the appropriate PWM value.
            if( abs( oldError - newError ) > 0.01 )                 //It is required for the difference between the errors to be more than a small number so that there are no errors with the precision limit of floating point variables.
            {
                oldTime = t.read();                                 //When a difference is apparent, the last time a revolution has been detected is updated.
                increasing = ( newError > oldError );
                if( ( oldTime > testDegree / 360 ) && increasing && decreasing )    //If the system has rotated for long enough and there is a suddent shift from increasing to decreasing, the time is recorded.
                {
                    timeData[index] = oldTime;
                    index++;
                }
                decreasing = !increasing;                           //Decreasing is updated after the comparison so that the point of inflection can be detected.
                oldError = newError;                                //Last error when a change in the position of the system has been detected is updated.
            }
            else if( ( t.read() - oldTime ) > stopTimeTreshold )
            {
                stopped = true;                                     //If the time until the last movement is sufficiently long, the system is deemed to have stopped.
            }
        }
        t.stop();           //The timer is stopped and reset.
        t.reset();
        stopMotor();
        wait(2);   
        encoder.reset();    //The encoder is reset two seconds after the stop command has been issued so that the last few degrees are not present in the recording of the next iteration.  
        
        Tc = 0.0;
        if( index >= 10 )   //If there have been enough time recordings of oscillation, Tc is calculated.
        {
            for( int i = 0; i < index - 1; i++ )
            {
                Tc += ( timeData[i+1] - timeData[i] );  //Differences are summed up, multiplied by two and divided by the data count to take the average.
            }
            Tc /= index;
            Tc *= 2;
        }     
    }while(Tc <= 0);        //The system keeps on increasing the proportional gain until Tc has been changed, which only occurs when the system has experienced an oscillation.
    
    kp = kc * 0.6;          //PID initial estimates are calculated through the ratios represented in Ziegler-Nichols Method and the found kp and Tc values.
    ki = 2 * kp / Tc;     
    kd = kp * Tc / 8;   
    serial.printf("Ziegler-Nichols Estimates: kp: %.3f, ki: %.5f, kd: %.5f\n", kp, ki, kd);
    serial.printf("Fine tuning in process:\n");
    
    
    //The integral and derivative parameters are changed by a small percentage until the system doesn't oscillate and keeps withing the overshoot requirement.
    kdIncrement = kd/100;   
    kiDecrement = ki/100;
    kd -= kdIncrement;
    ki += kiDecrement;
    index = 0;
    do
    {
        kd += kdIncrement;
        ki -= kiDecrement;
        index++;        //Keeping track of the amount of times the values have been changed to tune the system
        oldTime = 0;
        oldError = testDegree;
        stopped = false;
        integral = 0;   //The integral and the derivative are started off as zero so that they are only in effect after the first movement has been detected.
        derivative = 0;
        dt = 0;
        overshoot = 0;  //Overshoot is initially set to zero.
        t.start();
        while(t.read() < stabilizationPeriodAllowance && !stopped)
        {        
            degreesCovered = encoder.getPulses() * degreePerPulse;
            newError = testDegree - degreesCovered;
            newTime = t.read();
            if( abs( oldError - newError ) > 0.01 ) 
            {                
                dt = newTime - oldTime;     //dt represents the amount of time between the last two movements.
                oldTime = newTime;
                integral += newError * dt * ( abs(newError) < integralVicinityMax && abs(newError) > integralVicinityMin );     //The integral is a running sum of the error. It also multiplies with dt, and only adds new errors if the error is within the correct interval.
                derivative = ( newError - oldError ) / dt;  //The derivative is the difference between the last two error values, and is divided by dt.
                oldError = newError;
                if( overshoot > newError )  //Overshoot is updated if the system has surpassed the target degree more than it has ever had before.
                {
                    overshoot = newError;
                }
            }
            else if( ( newTime - oldTime ) > stopTimeTreshold )
            {
                stopped = true;
            }
            setSpeed( kp * newError + ki * integral + kd * derivative + pwmFrictionLimit );     //This time, all components of the PID are used to determine the PWM value.
        }
        t.stop();
        t.reset();
        stopMotor();        
        wait(2);   
        encoder.reset();       
        
    }while( ( overshoot < 0  ) && ( abs(overshoot) > overshootLimit ) && index < maximumTuningAttempts );   //The system keeps trying until the requirements are met or the maximum number of tuning attempts allowed is exceeded.
    
    
    //The system is now ready to function and it can receive user input.
    
    serial.printf("System has been fully tuned and is ready to use\n");
    serial.printf("Tuned parameters: kp: %.3f, ki: %.5f, kd: %.5f\n", kp, ki, kd);
    while(true)
    {
        float inputDegree;      //The target degree determined by the user.
        char inputChar;         //Each character sent through the serial communication is captured with this variable.
        char inputString[20];   //Holds the character array format of the inputted degree
        int digitIndex;         
        bool gotFloat = false;  //Indicates the completion of degree input.
        digitIndex = 0;
        serial.printf("Please enter degree: \n");
        while(!gotFloat)
        {        
            while(serial.readable())
            {
                inputChar = serial.getc();
                if(inputChar == '\n')                       //When the new line character is sent, it is implied that the user has finished typing the input degree.
                {                
                    inputDegree = atof(inputString);        //Array of character to float: atof function is used to retrieve the degree as a float.
                    gotFloat = true;                        
                    break;
                }
                else
                {
                    inputString[digitIndex] = inputChar;    //The char is added to the end of the character array.
                    digitIndex++;
                }            
            }     
        }
        serial.printf("Degree Received: %.5f\n", inputDegree);
    
        oldTime = 0;                                        //Required parameters are reset for the new run.
        oldError = inputDegree;
        stopped = false;
        integral = 0;
        derivative = 0;
        dt = 0;
        t.start();
        while(t.read() < stabilizationPeriodAllowance && !stopped)
        {        
            degreesCovered = encoder.getPulses() * degreePerPulse;
            newError = inputDegree - degreesCovered;
            newTime = t.read();
            if( abs( oldError - newError ) > 0.01 )
            {                
                dt = newTime - oldTime;
                oldTime = newTime;
                integral += newError * dt * ( abs(newError) < integralVicinityMax && abs(newError) > integralVicinityMin );
                derivative = ( newError - oldError ) / dt;
                oldError = newError;
            }
            else if( ( newTime - oldTime ) > stopTimeTreshold )
            {
                stopped = true;
            }
            setSpeed( kp * newError + ki * integral + kd * derivative + pwmFrictionLimit );
        }
        t.stop();
        t.reset();
        stopMotor();        
        wait(1);   
        serial.printf("Error: %.5f\n", newError);   //At the end, the error is reported.
        encoder.reset();       
    
    }
}
    

//Initializes the motor by setting the input pins, starting the PWM value with 0 and establishing the PWM frequency.
void initMotor(void)           
{
    in1 = 0;
    in2 = 0;
    pwmPin = 0.0f;              
    pwmPin.period(0.00033);    
}
//Sets the input pins so that the system turns clockwise
void turnCW(void)
{    
    in1 = 0;
    in2 = 1;
}
//Sets the input pins so that the system turns counter-clockwise
void turnCCW(void)
{    
    in2 = 0;
    in1 = 1;
}
//Sets the input pins so that the motor stops. Also resets the PWM pin.
void stopMotor(void)
{
    in1 = 0;
    in2 = 0;
    setSpeed(0.0);
}
//Changes the speed of the motor by adjusting the PWM pin with regards to the entered input. Negative numbers indicate counter-clockwise motion.
void setSpeed(float input)
{
    if (input >= 0)
    {
        turnCW();
        pwmPin = input;
    }
    else
    {
        turnCCW();
        pwmPin = -input;
    }
}
