// Necessary headers
#include "Arduino.h"
#include "ModemGPS.h"

// Include libraries
#include <TinyGPS.h>


// The constructor is just used for assigning member variables
ModemGPS::ModemGPS(const int gps_pps_pin)
{
    // Make the PPS pin an input
    pinMode(gps_pps_pin, INPUT);
}


// The initialize method is called once at startup
// It sets the GPS to use 10Hz update rate
void ModemGPS::init()
{
    // Setup serial port UART1 that is connected to the GPS
    Serial1.begin(9600);

    // Set the GPS to 115200 baud rate
    delay(2000);    
    Serial1.print("$PMTK251,115200*1F\r\n");
    //Serial1.print("$PMTK251,38400*27\r\n");
    delay(200);    
    Serial1.begin(115200);

    // Set GPS to 10Hz position update rate
    Serial1.print("$PMTK300,100,0,0,0,0*2C\r\n");
    //Serial1.print("$PMTK300,200,0,0,0,0*2F\r\n");
}


// Returns true if the GPS is locked, otherwise false
bool ModemGPS::locked()
{
    long latitude, longitude;
    unsigned long fix_age;
    gps.get_position(&latitude, &longitude, &fix_age);

    if(fix_age == TinyGPS::GPS_INVALID_AGE || fix_age > 10000)
        return false;
    else
        return true;
}


// Checks if any bytes are waiting to be received on the serial port
// If so, it sends these bytes to the GPS library
void ModemGPS::update()                     
{
    int incomingByte;

    // While there are bytes waiting to be processed
    while(Serial1.available() > 0)
    {
	    incomingByte = Serial1.read();
        gps.encode(incomingByte);
    }
}


// Writes the latest position and velocity information to the
// provided buffer
void ModemGPS::getData(volatile unsigned char* buf)
{
    // Raw measurements
    int lat, lon, altitude, altitude_m;
    unsigned long speed, lat_unsigned;
    unsigned short bearing, bearing_field;

    // Converted measurements
    unsigned char speed_kph, altitude_field;
 
    // +/- Lat and long in 100000ths of a degree
    gps.get_position((long*)&lat, (long*)&lon);
     
    // Speed in 100ths of a knot
    speed = gps.speed();
     
    // Course in 100ths of a degree
    bearing = gps.course();

    // +/- Altitude in 0.01 meters
    altitude = (int)gps.altitude();

    // Convert measurements
    altitude_m = altitude / 100;
    altitude_field = (unsigned char)(altitude_m & 0xFF);
    speed_kph = (unsigned char)((short)speed / 54);

    // Combine bearing and altitude into 1 field
    bearing_field = ( ((altitude_m >> 8) & 0x0F) << 12 ) | 
                    (unsigned short)( (bearing / 10) & 0x0FFF);

    // Convert latitude to sign-bit + 23 bits of measurement
    // Sign-bit = 1 indicates negative
    if(lat >= 0)
        lat_unsigned = (lat & 0x7FFFFF);
    else
    {
        lat_unsigned = ( (-lat) & 0x7FFFFF);
        lat_unsigned = lat_unsigned | 0x800000;
    }

    // Write position and velocity to the packet buffer
    // Teensy is little endian: word[0] = LSB, word[3] = MSB
    *(int*)(buf + 0) = lat_unsigned;
    *(int*)(buf + 3) = lon;
    *(short*)(buf + 7) = (short)(bearing_field);
    buf[9] = altitude_field;
    buf[10] = speed_kph;
}
