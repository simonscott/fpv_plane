/**
 * Notes:
 * 1.) The ADCs can sample at 100 000 times per second (i.e. can call analogRead 100k times per second)
 * 2.) The 6V current sensor is affected by the earth's field. Therefore, changing orientation of the sensor
 *     can change readings by up to 80mA.
 */


// Necessary headers
#include "Arduino.h"
#include "BatMonitor.h"


// The constructor is called once when the object is created
// It sets the ADC pins and sets up the ADC parameters
BatMonitor::BatMonitor(const int volt_6v_pin, const int curr_6v_pin, const int volt_12v_pin, const int curr_12v_pin)
{                
    // Setup ADC
    analogReadRes(ADC_RES);
    analogReadAveraging(128);

    // Set the ADC pins that we are going to use
    VOLT_6V_PIN = volt_6v_pin;
    CURR_6V_PIN = curr_6v_pin;
    VOLT_12V_PIN = volt_12v_pin;
    CURR_12V_PIN = curr_12v_pin;
}


// Measures the battery voltage and current, and writes the
// results to the provided buffer
void BatMonitor::getMeasurements(volatile unsigned char* buf)
{
    // Take ADC readings, apply calibration and convert readings to ints
    int volt_12v_raw = analogRead(VOLT_12V_PIN);
    float volt_12v_f = (float)(volt_12v_raw) * VOLT_12V_SCALE;    //mV
    unsigned int volt_12v = (unsigned int)(volt_12v_f);

    int curr_12v_raw = analogRead(CURR_12V_PIN);
    float curr_12v_f = (float)(curr_12v_raw) * CURR_12V_SCALE - CURR_12V_OFFSET;  //0.2 amps
    unsigned char curr_12v = (unsigned char)(curr_12v_f);
    
    int volt_6v_raw = analogRead(VOLT_6V_PIN);
    float volt_6v_f = (float)(volt_6v_raw) * VOLT_6V_SCALE - 500;   // 10mV scale, with 5V offset
    unsigned char volt_6v = (unsigned char)(volt_6v_f);

    int curr_6v_raw = analogRead(CURR_6V_PIN);
    float curr_6v_f = (float)(curr_6v_raw) * CURR_6V_SCALE - CURR_6V_OFFSET;  //10mA
    unsigned char curr_6v = (unsigned char)(curr_6v_f);

    // Write the measurements to the packet
    *(int*)(buf + 0) = volt_12v;
    buf[2] = curr_12v;
    buf[3] = volt_6v;
    buf[4] = curr_6v;
}

