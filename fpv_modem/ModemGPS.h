#ifndef MODEM_GPS_H
#define MODEM_GPS_H

#include <TinyGPS.h>

class ModemGPS
{
    public:
        // Public constructors and methods
        ModemGPS(const int gps_pps_pin);
        void init();
        bool locked();
        void update();
        void getData(volatile unsigned char* buf);

    private:
        // Private variables
        TinyGPS gps;
};

#endif
