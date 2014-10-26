#ifndef BAT_MONITOR_H
#define BAT_MONITOR_H

class BatMonitor
{
    public:
        // **** Public constructors and methods ****
        BatMonitor(const int volt_6v_pin, const int curr_6v_pin, const int volt_12v_pin, const int curr_12v_pin);
        void getMeasurements(volatile unsigned char* buf);

    private:
        // **** Private constants ****

        // Pin declarations
        int VOLT_6V_PIN;
        int CURR_6V_PIN;
        int VOLT_12V_PIN;
        int CURR_12V_PIN;

        // Scaling factors for voltage and current sensors
        const int ADC_RES           = 12;
        const float AN_VCC          = 3.325;

        const float VOLT_12V_SCALE  = (13.35/3.3) / pow(2, ADC_RES) * AN_VCC * 1000;
        const float CURR_12V_SCALE  = (1 / pow(2, ADC_RES) * AN_VCC) / 0.020 * 5;
        const float CURR_12V_OFFSET = AN_VCC/2 / 0.020 * 5 + 0.25 * 5;

        const float VOLT_6V_SCALE   = (13.29/3.28) / pow(2, ADC_RES) * AN_VCC * 100;
        const float CURR_6V_SCALE   = (1 / pow(2, ADC_RES) * AN_VCC) / 0.124 * 100;
        const float CURR_6V_OFFSET  = AN_VCC/2 / 0.124 * 100 + 8.5;
};

#endif
