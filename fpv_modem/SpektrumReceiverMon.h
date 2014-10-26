#ifndef SPEKTRUM_RECEIVER_MON_H
#define SPEKTRUM_RECEIVER_MON_H

// Parameter definitions
#define RSSI_LEN        30

// The class definition
class SpektrumReceiverMon
{
    public:
        // Public constructors and methods
        SpektrumReceiverMon(volatile int* time_counter_ms);
        void update();
        unsigned char getRSSI();

    private:
        // General private variables
        volatile int* timer;
        bool waiting_for_pkt;
        int bytes_read;
        int loss_write_pos;

        // Private variables to track RSSI status
        unsigned char signal_loss[RSSI_LEN];
        bool hold_condition;
        int old_holds;
        int new_holds;
        int old_frame_loss;
        int new_frame_loss;
        int old_main_rx_fades;
        int new_main_rx_fades;
        int old_sat_rx_fades;
        int new_sat_rx_fades;
};

#endif
