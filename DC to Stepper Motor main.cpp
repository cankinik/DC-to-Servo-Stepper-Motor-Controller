#include "mbed.h"
#include "QEI.h"


//Pins
DigitalOut in1(PD_3);           //Pin 13
DigitalOut in2(PB_14);          //Pin 12
PwmOut pwmPin(PB_15);           //Pin 11
InterruptIn interruptPin(PG_13);//Interrupt Digital Pin
InterruptIn button(PA_0);       //Interrupt Button
                                

//Functions
void initMotor(void);
void turnCW(void);
void turnCCW(void);
void stopMotor(void);
void setSpeed(float input);
void myInterrupt(void);
void buttonInterrupt(void);

//Entities
Serial serial(USBTX, USBRX);
QEI encoder (PG_10, PH_6, NC, 2, QEI::X2_ENCODING); //Pin 10 and 8 are used for encoder 
Timer t;

//Constants
const int stabilizationPeriodAllowance = 7;
const int stopTimeTreshold = 3;
const float degreePerPulse = 2.41;         
const float testDegree = 1080.0;      
const float integralVicinityMax = 20;
const float integralVicinityMin = degreePerPulse * 2;
const float pwmFrictionLimit = 0.2;       
const float kpStartingValue = 0.08;
const float kpIncrement = 0.01;
const float overshootLimit = degreePerPulse * 2;
const float maximumTuningAttempts = 1;
const float stepperIncrement = 30.0;

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

float inputDegree;

bool stopped;

//The comments until the stepper part of the code are exactly the same as the servo motor software, so they are not included here.   
int main()
{    
    serial.printf("Hello!\n");    
    initMotor();    
    
    
    kc = kpStartingValue;
    kc -= kpIncrement;
    t.reset();        
    
    do
    {
        kc += kpIncrement;  
        index = 0;
        oldTime = 0;
        oldError = testDegree;
        stopped = false;
        t.start();
        while(t.read() < stabilizationPeriodAllowance && !stopped)
        {
            degreesCovered = encoder.getPulses() * degreePerPulse;
            newError = testDegree - degreesCovered;
            setSpeed( kc * newError + pwmFrictionLimit );
            if( abs( oldError - newError ) > 0.01 )
            {
                oldTime = t.read();
                increasing = ( newError > oldError );
                if( ( oldTime > testDegree / 360 ) && increasing && decreasing )
                {
                    timeData[index] = oldTime;
                    index++;
                }
                decreasing = !increasing;
                oldError = newError;  
            }
            else if( ( t.read() - oldTime ) > stopTimeTreshold )
            {
                stopped = true;
            }
        }
        t.stop();
        t.reset();
        stopMotor();
        wait(2);   
        encoder.reset();       
        
        Tc = 0.0;
        if( index >= 10 )
        {
            for( int i = 0; i < index - 1; i++ )
            {
                Tc += ( timeData[i+1] - timeData[i] );
            }
            Tc /= index;
            Tc *= 2;
        }     
    }while(Tc <= 0);    
    
    kp = kc * 0.6;            
    ki = 2 * kp / Tc;     
    kd = kp * Tc / 8;   
    serial.printf("Ziegler-Nichols Estimates: kp: %.3f, ki: %.5f, kd: %.5f\n", kp, ki, kd);
    serial.printf("Fine tuning in process:\n");   
    
    kdIncrement = kd/100;
    kiDecrement = ki/100;
    kd -= kdIncrement;
    ki += kiDecrement;
    index = 0;
    do
    {
        kd += kdIncrement;
        ki -= kiDecrement;
        index++;
        oldTime = 0;
        oldError = testDegree;
        stopped = false;
        integral = 0;
        derivative = 0;
        dt = 0;
        overshoot = 0;
        t.start();
        while(t.read() < stabilizationPeriodAllowance && !stopped)
        {        
            degreesCovered = encoder.getPulses() * degreePerPulse;
            newError = testDegree - degreesCovered;
            newTime = t.read();
            if( abs( oldError - newError ) > 0.01 )
            {                
                dt = newTime - oldTime;
                oldTime = newTime;
                integral += newError * dt * ( abs(newError) < integralVicinityMax && abs(newError) > integralVicinityMin );
                derivative = ( newError - oldError ) / dt;
                oldError = newError;
                if( overshoot > newError )
                {
                    overshoot = newError;
                }
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
        wait(2);   
        encoder.reset();       
        
    }while( ( overshoot < 0  ) && ( abs(overshoot) > overshootLimit ) && index < maximumTuningAttempts );
    
    
    
    
    serial.printf("System has been fully tuned and is ready to use\n");
    serial.printf("Tuned parameters: kp: %.3f, ki: %.5f, kd: %.5f\n", kp, ki, kd);
    
    //------------------------------------------------------------------------
    //Here starts the part of the code that is different from the servo motor. 
    //------------------------------------------------------------------------
    
    interruptPin.rise(&myInterrupt);    //Pulse interrupt
    button.rise(&buttonInterrupt);      //Button interrupt
    
    
    oldTime = 0;
    oldError = inputDegree;
    stopped = false;
    integral = 0;
    derivative = 0;
    dt = 0;
    inputDegree = 0.0;
    t.start();
    while(true)         //The system no longer performs single runs, but rather always tries to achieve the input degree.
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
        setSpeed( kp * newError + ki * integral + kd * derivative + pwmFrictionLimit );    
    }
}
    
    
void initMotor(void)           
{
    in1 = 0;
    in2 = 0;
    pwmPin = 0.0f;              
    pwmPin.period(0.00033);    
}
void turnCW(void)
{
    in2 = 1;
    in1 = 0;
}
void turnCCW(void)
{
    in1 = 1;
    in2 = 0;
}
void stopMotor(void)
{
    in1 = 0;
    in2 = 0;
    setSpeed(0.0);
}
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
void myInterrupt()
{
    inputDegree += stepperIncrement;    //When a rising edge comes, the ISR increase both the input degree and also the error so that there will be no difference.
    oldError += stepperIncrement;
}

void buttonInterrupt()                  //When the button is pressed, the degree parameters will be reset.
{
    inputDegree = 0.0;
    encoder.reset();
    oldError = 0.0;
    newError = 0.0;
}
