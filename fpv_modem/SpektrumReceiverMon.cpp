/* Notes:
    1.) Use signal fades on single receivers to detect data loss. A frame loss is a much worse condition (means that frame could not be reconstructed at all.
    2.) Data packets are sent from receiver every 21.993 ms, and the packet is approx 1.36ms long.
    3.) Serial protocol: 115200 baud, 8 data bits, LSB first, no parity, 1 stop bit
    4.) Packet is 16 bytes:
        0       - 0x00 (for AR6210)
        1       - holds
        2       - frame loss (high byte)
        3       - frame loss (low byte)
        4       - main rx fades (high byte)
        5       - main rx fades (low byte)
        6       - satellite rx fades (high byte)
        7       - satellite rx fades (low byte)
        8-15    - unused

    5.) Fades indicates that a poor signal was detected on one of the antennas. The variable above is a count of the number of fades.
    6.) Frame loss means that the entire frame was discarded, as it couldn't be decoded from any of the antennas.
    7.) Holds is a count of the number of times that the receiver went into failsafe hold mode. This happens if 45 consecutive frame losses occur.
    8.) If a hold occurs, the frame loss counter is reset to zero.
    9.) Signal from Spektrum receiver is 3.3V
    10.) The signal loss is computed as sum of losses over the past 50 frames. The losses are weighted as follows:
        Hold = 255
        Frame loss = 127
        Receiver fade = 50
*/


// Necessary headers
#include "Arduino.h"
#include "SpektrumReceiverMon.h"


// The constructor runs once when the sketch starts
SpektrumReceiverMon::SpektrumReceiverMon(volatile int* time_counter_ms)
{
    // Initialize RSSI array to all zeroes
    int i;
    for(i = 0; i < RSSI_LEN; i++)
        signal_loss[i] = 0;

    // Initialize old counter variables
    old_holds = 0;
    old_frame_loss = 0;
    old_main_rx_fades = 0;
    old_sat_rx_fades = 0;
    hold_condition = false;

    // Initialize packet tracking variables
    waiting_for_pkt = true;
    bytes_read = 0;
    loss_write_pos = 0;

    // Serial port to spektrum receiver
    Serial2.begin(115200);

    // Store the timer and set it to zero
    timer = time_counter_ms;
    *timer = 0;
}


// Checks if any bytes are waiting to be received on the serial port
// If so, it sends these bytes to the GPS library
void SpektrumReceiverMon::update()                     
{
    char incomingByte;

    // Check for data from Spektrum
	if (Serial2.available() > 0)
    {
		incomingByte = Serial2.read();

        // If waiting for a new packet to start
        if(waiting_for_pkt)
        {
            if(*timer > 10)
            {
                waiting_for_pkt = false;
                bytes_read = 1;
            }
        }

        // Else we are busy reading a packet
        else
        {
            // Update the necessary variables
            if(bytes_read == 1)
                new_holds = incomingByte;
            else if(bytes_read == 2)
                new_frame_loss = (incomingByte << 8);
            else if(bytes_read == 3)
                new_frame_loss += incomingByte;
            else if(bytes_read == 4)
                new_main_rx_fades = (incomingByte << 8);
            else if(bytes_read == 5)
                new_main_rx_fades += incomingByte;
            else if(bytes_read == 6)
                new_sat_rx_fades = (incomingByte << 8);
            else if(bytes_read == 7)
                new_sat_rx_fades += incomingByte;

            // If we have read all of the important parts of the packet, update the RSSI
            if(bytes_read == 7)
            {
                int loss;
                bool new_hold = false;

                // If any holds, automatically a 255 loss
                if(new_holds != old_holds)
                {
                    new_hold = true;
                    loss = 255;
                }

                // Otherwise, compute loss
                else
                {
                    int frames_lost = (new_frame_loss > old_frame_loss) ? (new_frame_loss - old_frame_loss) : 0;
                    loss = frames_lost * 127 + (new_main_rx_fades - old_main_rx_fades) * 50 + (new_sat_rx_fades - old_sat_rx_fades) * 50;
                    if(loss > 230)
                        loss = 230;
                }
                
                // If we were previously under a hold condition
                if(hold_condition == true)
                {
                    // If we have some fading or frame loss, it means hold is over
                    if(loss > 0 && loss < 255)
                        hold_condition = false;
                }

                // If a new hold, record it
                if(new_hold)
                    hold_condition = true;

                signal_loss[loss_write_pos] = loss;

                // Update write pointer
                loss_write_pos += 1;

                if(loss_write_pos == RSSI_LEN)
                    loss_write_pos = 0;

                // Update the old/new registers
                old_holds = new_holds;
                old_frame_loss = new_frame_loss;
                old_main_rx_fades = new_main_rx_fades;
                old_sat_rx_fades = new_sat_rx_fades;

                waiting_for_pkt = true;
            }

            bytes_read++;
        }

        *timer = 0;
    }
}


// Writes the latest position and velocity information to the
// provided buffer
unsigned char SpektrumReceiverMon::getRSSI()
{
    int max_loss, i;
    max_loss = 0;
    
    // If under hold condition
    if(hold_condition)
        return 0;

    // Otherwise, not under hold
    for(i = 0; i < RSSI_LEN; i++)
    {
        if(signal_loss[i] > max_loss)
            max_loss = signal_loss[i];
    }

    return (255 - max_loss);
}
