/*
 * A modem for the teensy ARM board
 * The program generates an artificial data stream. This data stream
 * is then encoded using a sequence of tones. This is done
 * using the FTM1 module to generate PWM signals on pin 4
 *
 * The video transmitter has a usable bandwidth from 500Hz to 10kHz.
 *
 * Start of packet is indicated by a chirp from 1000Hz to 6000Hz.
 * This chirp is 5ms long.
 *
 * The packet itself is approximately 20 bytes long. Each byte is broken
 * down into 2-bit words, and transmitted using the tone set below"
 * 2667Hz - 00
 * 4000Hz - 01
 * 5333Hz - 10
 * 6667Hz - 11
 *
 * Each tone should be transmitted for at least 4 cycles.
 * This means that each symbol should be transmitted for 4 / 2667 = 1.5ms
 * Therefore, a symbol rate of 1/0.0015 = 667 symbols/sec is possible.
 * This gives bit rate of 1333bps, or 166Bps.
 * TO DO:
 * 1.) Fine tune the PWM tune parameter
 */


// Libraries and headers to include
#include <TinyGPS.h>
#include "ModemGPS.h"
#include "BatMonitor.h"
#include "SpektrumReceiverMon.h"


/******************************************************
 * Constant definitions
 ******************************************************
 */

// Pins
const int LED_PIN       = 13;   // LED turns on to indicate power
const int PWM_PIN       = 4;    // PWM output appears on this pin
const int GPS_PPS_PIN   = 2;
const int VOLT_6V_PIN   = 14;
const int CURR_6V_PIN   = 15;
const int VOLT_12V_PIN  = 16;
const int CURR_12V_PIN  = 17;

// PWM timing parameters
const int pwm_clk = 93750;
const int pwm_tune = 0;
const int pwm_res = 9;

// Tone parameters
#define NO_TONE         0
#define FIXED_TONE		1
#define CHIRP_TONE		2
const int symbol_length_us = 1500;
const int tone_table[] = {2667, 4000, 5333, 6667};
const int tx_msg_len = 20;

// Chirp parameters
const int chirp_start_f = 500; //Hz
const int chirp_stop_f = 6000;  //Hz
const int chirp_bw = chirp_stop_f - chirp_start_f;
const int chirp_duration = symbol_length_us * 4;   //usec
const int chirp_delta_f = (int)( (float)chirp_bw * (1000000.0 / (float)(pwm_clk * chirp_duration) ) );
const long tune_delta = pow(2,32) / (pwm_clk + pwm_tune) * chirp_delta_f;

// CRC parameters and macros
#define CRC_POLY    0x8005
#define POLY_WD     16
#define TOP_BIT     (1 << (POLY_WD - 1))

// Debug parameters
#define USE_TEST_DATA   0

// Table of 256 sine values (one sine period), stored in flash memory
const unsigned short sine256[256]  =
{
255,260,265,270,275,280,285,290,295,300,305,309,314,319,324,328,333,338,342,347,351,356,360,364,368,372,377,381,384,388,392,396,399,403,406,409,
413,416,419,422,425,427,430,432,435,437,439,441,443,445,447,449,450,452,453,454,455,456,457,457,458,458,459,459,459,459,459,458,458,457,457,456,
455,454,453,452,450,449,447,445,443,441,439,437,435,432,430,427,425,422,419,416,413,409,406,403,399,396,392,388,384,381,377,372,368,364,360,356,
351,347,342,338,333,328,324,319,314,309,305,300,295,290,285,280,275,270,265,260,255,250,245,240,235,230,225,220,215,210,205,201,196,191,186,182,
177,172,168,163,159,154,150,146,142,138,133,129,126,122,118,114,111,107,104,101,97,94,91,88,85,83,80,78,75,73,71,69,67,65,63,61,60,58,57,56,55,
54,53,53,52,52,51,51,51,51,51,52,52,53,53,54,55,56,57,58,60,61,63,65,67,69,71,73,75,78,80,83,85,88,91,94,97,101,104,107,111,114,118,122,126,129,
133,138,142,146,150,154,159,163,168,172,177,182,186,191,196,201,205,210,215,220,225,230,235,240,245,250
};

// Table of CRC lookup values for generator polynomial 0x8005
const unsigned short crc_table[256] =
{
0, 32773, 32783, 10, 32795, 30, 20, 32785, 32819, 54, 60, 32825, 40, 32813, 32807, 34, 32867, 102, 108, 32873, 120, 32893, 32887, 114, 80, 32853, 32863,
90, 32843, 78, 68, 32833, 32963, 198, 204, 32969, 216, 32989, 32983, 210, 240, 33013, 33023, 250, 33003, 238, 228, 32993, 160, 32933, 32943, 170, 32955,
190, 180, 32945, 32915, 150, 156, 32921, 136, 32909, 32903, 130, 33155, 390, 396, 33161, 408, 33181, 33175, 402, 432, 33205, 33215, 442, 33195, 430, 420,
33185, 480, 33253, 33263, 490, 33275, 510, 500, 33265, 33235, 470, 476, 33241, 456, 33229, 33223, 450, 320, 33093, 33103, 330, 33115, 350, 340, 33105,
33139, 374, 380, 33145, 360, 33133, 33127, 354, 33059, 294, 300, 33065, 312, 33085, 33079, 306, 272, 33045, 33055, 282, 33035, 270, 260, 33025, 33539,
774, 780, 33545, 792, 33565, 33559, 786, 816, 33589, 33599, 826, 33579, 814, 804, 33569, 864, 33637, 33647, 874, 33659, 894, 884, 33649, 33619, 854, 860,
33625, 840, 33613, 33607, 834, 960, 33733, 33743, 970, 33755, 990, 980, 33745, 33779, 1014, 1020, 33785, 1000, 33773, 33767, 994, 33699, 934, 940, 33705,
952, 33725, 33719, 946, 912, 33685, 33695, 922, 33675, 910, 900, 33665, 640, 33413, 33423, 650, 33435, 670, 660, 33425, 33459, 694, 700, 33465, 680, 33453,
33447, 674, 33507, 742, 748, 33513, 760, 33533, 33527, 754, 720, 33493, 33503, 730, 33483, 718, 708, 33473, 33347, 582, 588, 33353, 600, 33373, 33367, 594,
624, 33397, 33407, 634, 33387, 622, 612, 33377, 544, 33317, 33327, 554, 33339, 574, 564, 33329, 33299, 534, 540, 33305, 520, 33293, 33287, 514
};


/******************************************************
 * Global variables
 ******************************************************
 */

// The buffer that stores message to be transmitted
// After setting tx_msg and len, the main loop must set "pending_msg"
// to true.
volatile unsigned char tx_buffer[2][32];
volatile int curr_tx_buf;
bool pkt_created;
unsigned char pkt_count;

// Note that global variables used inside ISRs are declared as volatile

// Variables for the FTM1 ISR
volatile int tone_type;
volatile byte sine_idx;           // for indexing the sine table
volatile unsigned long ph_accu;   // phase accumulator
volatile unsigned long tune_word; // DDS tuning word

// Variables for the PIT ISR
IntervalTimer symbol_timer;
volatile bool pending_msg;        // cleared once complete msg sent
volatile int curr_byte;
volatile int curr_bit_pos;

// Variables for the millisecond ISR
IntervalTimer ms_timer;
volatile int pkt_tx_counter_ms;
volatile int led_counter_ms;
volatile int spektrum_counter_ms;

// GPS, Battery Monitor and RSSI objects
ModemGPS gps(GPS_PPS_PIN);
BatMonitor bat_mon(VOLT_6V_PIN, CURR_6V_PIN, VOLT_12V_PIN, CURR_12V_PIN);
SpektrumReceiverMon spektrum_mon(&spektrum_counter_ms);


/******************************************************
 * The setup method, which runs once at startup
 ******************************************************
 */
void setup()
{                
    // Configure the digital output pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);

    // Configure PWM frequency (on pin 3)
    // Also set PWM write resolution
    analogWriteFrequency(PWM_PIN, pwm_clk);
    analogWriteRes(pwm_res);

    // Initialize the analog output to zero
    analogWrite(PWM_PIN, 0); 

    // Initialize the GPS to run at 10Hz
    gps.init();

    // Setup the transmit message buffers
    pending_msg = false;
    curr_byte = -1;
    curr_bit_pos = 0;
    curr_tx_buf = 0;
    pkt_tx_counter_ms = 0;
    led_counter_ms = 0;
    pkt_created = false;
    pkt_count = 0;

    // Setup initial tone
    set_tone(NO_TONE);

    // Enable interrupts for FTM1 counter on overflow
    NVIC_ENABLE_IRQ(IRQ_FTM1);
    FTM1_SC |= FTM_SC_TOIE;
    FTM1_SC &= ~FTM_SC_TOF;

    // Setup the perdiodic interrupt timer to run every 1.5ms
    symbol_timer.begin(symbol_timer_isr, symbol_length_us);

    // Setup the millisecond interrupt timer to run every 1ms
    ms_timer.begin(ms_timer_isr, 1000);
}


/******************************************************
 * The main software loop
 ******************************************************
 */
void loop()                     
{
    // Check for new data from the GPS
    gps.update();

    // Check for new data from the Spektrum receiver data port
    spektrum_mon.update();

    // Create the packet
    if(pkt_tx_counter_ms >= 90 && !pkt_created)
    {
        // If test mode
        if(USE_TEST_DATA)
        {
            // Write a simple message to the transmit buffer
            sprintf((char*)tx_buffer[curr_tx_buf^1], "This is a message.");
        }

        // Else creating real data
        else
        {
            // Write packet counter
            tx_buffer[curr_tx_buf^1][0] = pkt_count;

            // Write battery info to packet
            bat_mon.getMeasurements(tx_buffer[curr_tx_buf^1] + 1);

            // Write RC receiver signal strength to packet
            tx_buffer[curr_tx_buf^1][6] = spektrum_mon.getRSSI();
    
            // Write GPS data to packet
            if(gps.locked())
                gps.getData(tx_buffer[curr_tx_buf^1] + 7);
            else
            {
                tx_buffer[curr_tx_buf^1][14] = 0xFF;
                tx_buffer[curr_tx_buf^1][15] = 0xFF;
            }

            pkt_count++;
        }

        // Append the CRC word
        unsigned short crc = gen_crc(tx_buffer[curr_tx_buf^1], tx_msg_len-2);
        tx_buffer[curr_tx_buf^1][tx_msg_len-2] = (unsigned char)(crc >> 8);
        tx_buffer[curr_tx_buf^1][tx_msg_len-1] = (unsigned char)(crc & 0x00FF);

        pkt_created = true;
    }

    // Send the packet
    else if(pkt_tx_counter_ms >= 132 && pkt_created && !pending_msg)
    {
        pkt_tx_counter_ms = 0;
        curr_tx_buf ^= 1;
        pending_msg = true;
        pkt_created = false;
    }

    // Flash every 0.5 sec
    if(led_counter_ms == 500)
        digitalWrite(LED_PIN, HIGH);
    else if(led_counter_ms >= 1000)
    {
        led_counter_ms = 0;
        digitalWrite(LED_PIN, LOW);
    }
}


/******************************************************
 * The Symbol Timer ISR
 * Runs every 1.5ms
 * Each time it runs, it changes the current tone.
 ******************************************************
 */
void symbol_timer_isr(void)
{
    // If a message is waiting to be sent
    if(pending_msg)
    {

        // If start of a new packet, send start chirp
        if(curr_byte == -1 && curr_bit_pos == 0)
		{
		    set_tone(chirp_start_f);
            tone_type = CHIRP_TONE;
            ph_accu = 0;
		}

        // Else send next pair of bits in current tone
        else if(curr_byte >= 0)
        {
            unsigned char bits = (tx_buffer[curr_tx_buf][curr_byte] >> curr_bit_pos) & 0x03;
			tone_type = FIXED_TONE;
            set_tone(tone_table[bits]);
        }

        curr_bit_pos += 2;

        // Logic to increment to next byte and/or message
        if(curr_bit_pos == 8)
        {
            curr_bit_pos = 0;
            curr_byte += 1;

            if(curr_byte == tx_msg_len)
            {
                curr_byte = -1;
                pending_msg = false;
            }
        }
    }

    // Else set tone to DC
    else
	{
		tone_type = FIXED_TONE;
        set_tone(NO_TONE);
	}
}


/******************************************************
 * The millisecond ISR
 * Runs every 1ms
 * Each time it runs, it increments the timers
 ******************************************************
 */
void ms_timer_isr(void)
{
    pkt_tx_counter_ms++;
    led_counter_ms++;
    spektrum_counter_ms++;
}


/******************************************************
 * Function to set the tone currently being generated.
 * Parameter 'tone' is in Hz
 ******************************************************
 */
void set_tone(int tone)
{
    tune_word = pow(2,32) * tone / (pwm_clk + pwm_tune); // calulate DDS new tuning word 
}


/******************************************************
 * Function to generate the CRC word
 ******************************************************
 */
unsigned short gen_crc(volatile unsigned char* message, int message_len)
{
    unsigned char data;
    unsigned short remainder = 0;
    int byte;

    // Divide the message by the polynomial, a byte at a time.
    for(byte = 0; byte < message_len; byte++)
    {
        data = message[byte] ^ (remainder >> (POLY_WD - 8));
        remainder = crc_table[data] ^ (remainder << 8);
    }

    // The final remainder is the CRC.
    return remainder;
}


/******************************************************
 * The FTM1 ISR
 * This ISR is responsible for generating the sine waves
 ******************************************************
 */
void ftm1_isr(void)
{
    // Re-enable interrupt (clear TOF flag)
    FTM1_SC &= ~FTM_SC_TOF;

	ph_accu = ph_accu + tune_word;  // phase accu with 32 bits
	sine_idx = ph_accu >> 24;       // use upper 8 bits of phase accum to idx sine table
	
    // Write new value to PWM comparator
    analogWrite(PWM_PIN, sine256[sine_idx]);  

	// If a chirp, we need to calculate the new tuning word
	if(tone_type == CHIRP_TONE)
        tune_word = tune_word + tune_delta;
}
