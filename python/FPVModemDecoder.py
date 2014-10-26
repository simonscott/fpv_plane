"""
FPV Modem Decoder
Decodes the audio stream from a USB audio-input dongle.
"""

import pyaudio
import wave
import time
import numpy
from scipy import signal
import matplotlib.pyplot as plt
import wx
from crcmod import mkCrcFun
import struct
from math import sin, cos, radians, asin, atan2, pi, sqrt
import datetime

import FPVModemWinV2


# Compilation parameters
use_recorded_samples = False
log_en = True


"""
Structure to store the decoded contents of a packet
"""
class FPVModemPacket:
    pkt_cntr = 0
    bat_volt_12v = 0
    bat_curr_12v = 0
    bat_volt_6v = 0
    bat_curr_6v = 0
    spektrum_rssi = 0
    latitude = 0
    longitude = 0
    altitude = 0
    speed = 0
    bearing = 0


"""
Structure to store the computed airplane characteristics
"""
class FPVComputedProperties:
    home_lat = 0
    home_lon = 0
    home_alt = 0
    distance = 0
    dir_home = 0

    filt_altitude = 0
    alt_valid = False
    
    bat_cap_used_12v = 0
    bat_cap_used_6v = 0


"""
The actual real-time FPV modem decoder
"""
class FPVModemDecoder:


    """
    Constructor
    """
    def __init__(self, updateDisplayFunc, updateGUIStatusFunc):


        # Define Constants
        self.SAMPLE_WIDTH = 4
        self.NUM_CHAN = 1
        self.SAMP_RATE = 48000
        self.SAMPS_PER_BUF = (int)(self.SAMP_RATE * 0.040)      # 40ms of data per buffer
        self.BUF_SIZE = (int)(self.SAMP_RATE * 0.800)           # the ring buffer stores 800ms of samples
        self.OSC_CORR_FACTOR = 0.95                             # corrects for frequency offset in Teensy xtal
        self.PEAK_AVG_THRES = 40                                # threshold for peak correlation / avg pwr. Noise=[16-22], sig=[80-87]
        self.SAMP_CNT_150MS = (int)(0.150*self.SAMP_RATE)

        # Audio and GUI member variables
        self.updateDisplay = updateDisplayFunc
        self.updateGUIStatus = updateGUIStatusFunc
        self.updateGUIStatus(FPVModemWinV2.STATE_NO_SIG)
        self.audio = pyaudio.PyAudio()

        # Create buffer variables
        self.buf = numpy.zeros(self.BUF_SIZE, dtype=numpy.float32)
        self.write_ptr = 0
        self.read_ptr = 0
        self.buf_cnt = 0

        # Packet parameters
        self.symbol_len = 0.0015
        self.pkt_len_bytes = 20
        self.pkt_len_symbols = self.pkt_len_bytes * 4

        # Chirp parameters
        self.chirp_start_f = 500 * self.OSC_CORR_FACTOR
        self.chirp_stop_f = 6000 * self.OSC_CORR_FACTOR
        self.chirp_duration = self.symbol_len * 4
        self.matched_filt = self.create_matched_filter()

        # Create function to generate CRC
        self.compute_crc = mkCrcFun(0x18005, 0, rev=False, xorOut=0)
    
        # Control member variables
        self.pkt_start_aligned = False
        self.pkt_d_start_aligned = False

        # Packet member variables
        self.total_pkts_lost = 0
        self.contig_bad_crcs = 0
        self.packet = FPVModemPacket()

        # Variables to compute more complex airplane properties
        self.fpv_props = FPVComputedProperties()
        self.waiting_for_home = False
        self.alt_filter = []

        # Control the logging of packets
        if(log_en):
            self.log_file = open('fpv_log_' + datetime.datetime.strftime(datetime.datetime.today(), '%Y%m%d_%H%M') + '.txt', 'w')

        # If we are taking live samples
        if(not use_recorded_samples):

            # Figure out which audio device to use
            dev_index = None

            for d in range(0, self.audio.get_device_count()):

                print self.audio.get_device_info_by_index(d)
                print " "

                #if self.audio.get_device_info_by_index(d)['name'].startswith("Cx231xx Audio: Conexant cx231xx Capture"):
                if self.audio.get_device_info_by_index(d)['name'] == "default":
                    dev_index = d

            print "Using device num: ", dev_index

            # Open the audio stream
            self.stream = self.audio.open(
                            format = pyaudio.paFloat32,
                            channels = self.NUM_CHAN,
                            rate = self.SAMP_RATE,
                            frames_per_buffer = self.SAMPS_PER_BUF,
                            input_device_index = dev_index,
                            input = True,
                            output = False,
                            stream_callback = self.audio_frame_callback)

            self.stream.start_stream()

        # Else we are using recorded samples for testing purposes
        else:

            self.wavefile = wave.open("../../modem_data_samples.wav", 'rb')
            #self.wavefile = wave.open("../../modem_noise_samples.wav", 'rb')

            # Open the audio stream
            self.stream = self.audio.open(
                            format = pyaudio.paInt16,
                            channels = self.NUM_CHAN,
                            rate = self.SAMP_RATE,
                            frames_per_buffer = self.SAMPS_PER_BUF,
                            input = True,
                            output = False,
                            stream_callback = self.audio_frame_callback)

            self.stream.start_stream()


    """
    Destructor
    """
    def __del__(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()


    """ 
    Callback function:
    This function is called whenever a full frame of audio has been recorded
    """
    def audio_frame_callback(self, in_data, frame_count, time_info, status):
        
        # If we are using live data
        if(not use_recorded_samples):
            in_data_f = numpy.fromstring(in_data, 'Float32')
        else:
            in_data_16b = self.wavefile.readframes(frame_count)
            in_data_f = numpy.fromstring(in_data_16b, 'Int16').astype(numpy.float32)

        # Process the samples
        try:
            self.process_samples(in_data_f)

        # If something goes wrong, just erase the buffer
        except:
            self.write_ptr = 0
            self.read_ptr = 0
            self.buf_cnt = 0

        return (None, pyaudio.paContinue)


    """
    Function to actually process the most recently received set of samples.
    These samples are added to a ring buffer, and if enough samples have been
    received, the samples are processed and the packet decoded.
    """
    def process_samples(self, samples):

        # If buffer is full
        if self.buf_cnt + len(samples) > self.BUF_SIZE:

            # Discard old samples
            mark_samples_read(self.buf_cnt + self.SAMPS_PER_BUF - self.BUF_SIZE)

        # Add samples to ring buffer
        # If the samples will write past end of buffer
        if(self.write_ptr + self.SAMPS_PER_BUF > self.BUF_SIZE):
            space_end = self.BUF_SIZE - self.write_ptr
            self.buf[self.write_ptr : ] = samples[ : space_end]
            self.buf[ : (self.SAMPS_PER_BUF - space_end)] = samples[space_end : ]
    
        # Else just write samples contiguously
        else:
            self.buf[self.write_ptr : self.write_ptr + self.SAMPS_PER_BUF] = samples

        self.write_ptr = (self.write_ptr + self.SAMPS_PER_BUF) % self.BUF_SIZE
        self.buf_cnt += self.SAMPS_PER_BUF
        self.pkt_d_start_aligned = False

        #############################################################################
        # If we haven't aligned the packets to the beginning of the buffer
        #############################################################################
        if(not self.pkt_start_aligned):
            
            # Wait until we have 300ms worth of samples
            if self.buf_cnt < self.SAMP_RATE * 0.300:
                return

            #time_axis = numpy.linspace(0, self.buf_cnt / float(self.SAMP_RATE), self.buf_cnt)
            #plt.subplot(2, 1, 1)
            #plt.plot(time_axis, numpy.take(self.buf, numpy.arange(self.read_ptr, self.read_ptr+self.buf_cnt), mode='wrap'), 'g')

            # Apply matched filter
            filter_out = numpy.convolve( numpy.take(self.buf, numpy.arange(self.read_ptr, self.read_ptr+self.buf_cnt), mode='wrap') , self.matched_filt )
            time_axis = numpy.arange(0, len(filter_out) / float(self.SAMP_RATE), 1/float(self.SAMP_RATE))
            #plt.subplot(2, 1, 2)
            #plt.plot(filter_out, 'b')
            #plt.show()

            # Figure out if we have actually detected a signal
            # We do this by comparing the peak of the matched filter to the average power
            filter_out_pwr = numpy.square(filter_out)
            filter_out_avg_pwr = numpy.mean(filter_out_pwr)
            filter_out_max_pwr = max(filter_out_pwr)
            max_avg_ratio = filter_out_max_pwr / filter_out_avg_pwr
            print "Avg pwr = ", filter_out_avg_pwr, " max pwr = ", filter_out_max_pwr, " ratio = ", max_avg_ratio

            # If we don't detect a signal, discard all samples and return
            if(max_avg_ratio < self.PEAK_AVG_THRES):
                self.mark_samples_read(self.buf_cnt)
                return

            # Find the max value and set the filter threshold for peak detection
            self.filt_threshold = filter_out_max_pwr * 0.75

            # Find the position of the first peak in the matched filter output
            peak_idx = self.find_first_mf_peak(filter_out_pwr)
            print "Peak idx=", peak_idx, " Total buf size=", self.buf_cnt,"\n"

            if(peak_idx == -1):
                self.mark_samples_read(self.buf_cnt)
                return

            # Move the read ptr to the begin of the data section of the packet
            self.mark_samples_read(peak_idx - 2)

            self.pkt_start_aligned = True
            self.pkt_d_start_aligned = True
            self.updateGUIStatus(FPVModemWinV2.STATE_NORMAL)

        #############################################################################
        # By now we have already aligned the next packet with the beginning of the circular buffer
        #############################################################################

        # Wait for 150ms of data (just more than 1 packet)
        if(self.buf_cnt < self.SAMP_CNT_150MS):
            return

        time1 = time.clock()

        # If the data portion of the packet is not perfectly aligned with the beginning
        # of the circular buffer
        if(not self.pkt_d_start_aligned):

            filter_out = numpy.convolve( numpy.take(self.buf, numpy.arange(self.read_ptr, self.read_ptr + self.SAMP_CNT_150MS), mode='wrap') ,
                                            self.matched_filt )

            filter_out_pwr = numpy.square(filter_out)
            filter_out_avg_pwr = numpy.mean(filter_out_pwr)
            filter_out_max_pwr = numpy.max(filter_out_pwr)
            max_avg_ratio = filter_out_max_pwr / filter_out_avg_pwr
            print "Avg/max ratio = ", max_avg_ratio

            # If we don't detect a signal, discard all samples and return
            if(max_avg_ratio < self.PEAK_AVG_THRES or filter_out_max_pwr < self.filt_threshold):
                self.mark_samples_read(self.SAMP_CNT_150MS)
                self.pkt_start_aligned = False
                self.updateGUIStatus(FPVModemWinV2.STATE_NO_SIG)
                return

            # Find the position of the first peak in the matched filter output
            peak_idx = self.find_first_mf_peak(filter_out_pwr)

            # In case the peak is not at the beginning of the buffer (which is where it should be),
            # make sure that we have enough samples to obtain the entire packet
            if(self.buf_cnt - peak_idx < (int)(self.pkt_len_symbols * self.symbol_len * self.SAMP_RATE)):
                self.pkt_start_aligned = False
                print "Buf too small. Peak idx =", str(peak_idx), " . Must be ", str((int)(self.pkt_len_symbols * self.symbol_len * self.SAMP_RATE)), "\n"
                self.updateGUIStatus(FPVModemWinV2.STATE_NO_SIG)
                return

            # Move the read ptr to the begin of the data section of the packet
            self.mark_samples_read(peak_idx - 2)

        #############################################################################
        # The data section of the packet is now perfectly aligned with begin of buffer
        #############################################################################

        # Divide packet up into symbol windows
        samps_per_symbol = (int)(self.symbol_len * self.SAMP_RATE)
        symbol_samps = numpy.reshape(
                                    numpy.take( self.buf, numpy.arange(self.read_ptr,
                                                self.read_ptr + self.pkt_len_symbols * samps_per_symbol), mode='wrap'),
                                    (self.pkt_len_symbols, samps_per_symbol),
                                    order='C' )
        
        # Take FFT inside each symbol window
        # The FFT indices correspond to following frequencies:
        # [0, 666, 1333, 2000, 2667, 3333, 4000, 4667, 5333, 6000, 6667, 7333, etc]
        # The corresponding symbols are given in table below:
        symbol_table = numpy.ones(samps_per_symbol/2) * -1
        symbol_table[0:11] = [-1, -1, -1, -1, 0, -1, 1, -1, 2, -1, 3];

        symbols_fft = [abs(numpy.fft.rfft(sym)) for sym in symbol_samps]

        #freq = numpy.linspace(0, self.SAMP_RATE/2, len(symbols_fft[0]))
        #plt.subplot(2, 1, 1)
        #plt.plot(symbol_samps[0, :], 'b')
        #plt.subplot(2, 1, 2)
        #plt.plot(freq, symbols_fft[0], 'b')
        #plt.show()

        # Compute the corresponding symbol
        symbol_freq_idx = [numpy.argmax(fft) for fft in symbols_fft]
        symbols = [symbol_table[idx] for idx in symbol_freq_idx]

        # If any symbols are "-1", immediately throw the packet out
        if(-1 in symbols):

            print "ERROR: packet contains corrupted symbols"
            self.total_pkts_lost += 1
            self.contig_bad_crcs += 1

        else:

            # Convert each symbol to bytes
            pkt_bytes = [(s[3]*64 + s[2]*16 + s[1]*4 + s[0]) for s in numpy.reshape(symbols, (self.pkt_len_bytes, 4), order='C')]

            # Decode packet
            self.decode_packet(pkt_bytes)

        # Refresh GUI        
        self.updateDisplay( (int)(self.packet.spektrum_rssi / 255.0 * 100.0), self.fpv_props.filt_altitude,
                            self.fpv_props.distance, self.packet.speed, self.fpv_props.dir_home, self.total_pkts_lost, not self.waiting_for_home)

        # Mark packet as read in circular buffer
        self.mark_samples_read(self.pkt_len_symbols * samps_per_symbol)

        time2 = time.clock()
        print "Packet processing time = ", str((time2-time1)*1000.0), " ms"
        print ""


    """
    Create the matched filter to detect the beginning of a packet
    """
    def create_matched_filter(self):

        matched_filt_t = numpy.linspace(0, self.chirp_duration - 1/self.SAMP_RATE, self.chirp_duration*self.SAMP_RATE)
        matched_filt = signal.chirp(matched_filt_t, self.chirp_start_f, self.chirp_duration - 1/self.SAMP_RATE, self.chirp_stop_f);
        matched_filt = matched_filt * numpy.linspace(1, 0.5, len(matched_filt_t));
        matched_filt = matched_filt[::-1];
        return matched_filt


    """
    Mark the specified number of samples as read, starting from the read pointer position
    """
    def mark_samples_read(self, num_samples):

        self.read_ptr = (self.read_ptr + num_samples) % self.BUF_SIZE
        self.buf_cnt -= max(num_samples, 0)


    """
    Returns the index of the first peak in the matched filter output.
    The parameter is the POWER of the matched filter output
    Note that the list "filter_out_pwr" is destroyed by this function.
    """
    def find_first_mf_peak(self, filt_out_pwr):

        # Each peak is actually a set of multiple peaks, so we take the largest
        # peak within a 10ms window.
        # We do this for the first peak only, as we don't care about the rest of the packets
        window_length = 0.010
        peaks_idx = 0
        while(peaks_idx < len(filt_out_pwr)):
            
            # If this is the beginning of a peak
            if(filt_out_pwr[peaks_idx] > self.filt_threshold):

                end_window = min(peaks_idx + self.SAMP_RATE * window_length, len(filt_out_pwr))
                max_idx = numpy.argmax(filt_out_pwr[peaks_idx:end_window])
                return peaks_idx + max_idx
              
            # Else not near a peak
            else:
               peaks_idx = peaks_idx + 1

        return -1


    """
    Decodes a modem packet.
    This function also processes the packet information to compute distance to aircraft, etc
    Finally, the function calls the necessary GUI functions to update the display.
    """
    def decode_packet(self, pkt):

        # Compute CRC
        pkt_str = struct.pack("<"+str(self.pkt_len_bytes)+"B", *pkt)
        crc_check = self.compute_crc(pkt_str)

        # If the CRC is valid
        if(crc_check == 0):
                        
            last_valid_pkt_cntr = self.packet.pkt_cntr

            # Decode fields
            # Note: the packet is in little-endian format
            (   self.packet.pkt_cntr, bat_volt_12v_raw, bat_curr_12v_raw, bat_volt_6v_raw, bat_curr_6v_raw,
                self.packet.spektrum_rssi, lat_low_word, lat_high_byte, longitude_raw, bearing_raw, altitude_raw,
                self.packet.speed, crc) = \
                struct.unpack("<BHBBBBHBiHBBH", pkt_str)
            
            # Fields requiring post-processing
            self.packet.bat_volt_12v = bat_volt_12v_raw / 1000.0
            self.packet.bat_curr_12v = bat_curr_12v_raw / 5.0
            self.packet.bat_volt_6v = (bat_volt_6v_raw / 100.0) + 5.0
            self.packet.bat_curr_6v = bat_curr_6v_raw * 10
            self.packet.longitude = longitude_raw / 100000.0

            # Combine and sign-extend altitude fields
            self.packet.altitude = ((bearing_raw & 0xF000) >> 4) | altitude_raw
            if(bearing_raw & 0x8000 != 0):
                self.packet.altitude = self.packet.altitude - 2**12;
        
            # Correctly decode latitude from magnitude and sign
            lat_mag = ( ((lat_high_byte & 0x7F) << 16) + lat_low_word )
            if(lat_high_byte & 0x80 == 0):
                self.packet.latitude = lat_mag / 100000.0
            else:
                self.packet.latitude = -1 * lat_mag / 100000.0

            # Determine if GPS lock from bearing
            if(bearing_raw == 0xFFFF):
                self.updateGUIStatus(FPVModemWinV2.STATE_NO_LOCK)
            else:
                self.updateGUIStatus(FPVModemWinV2.STATE_NORMAL)
                self.packet.bearing = bearing_raw / 10.0
            
            # Only do further processing if we have a GPS lock
            if(bearing_raw != 0xFFFF):

                # Append the altitude to the altitude filter
                self.alt_filter.append(self.packet.altitude)
            
                # Filter the altitude over the past 2 measurements
                if(len(self.alt_filter) == 2):
                    self.fpv_props.filt_altitude = sum(self.alt_filter) / 2 - self.fpv_props.home_alt
                    self.fpv_props.alt_valid = True
                    self.alt_filter.pop(0)

                # If plane still on ground, record starting altitude and position
                if(self.waiting_for_home and self.fpv_props.alt_valid):

                    self.fpv_props.home_lat = self.packet.latitude
                    self.fpv_props.home_lon = self.packet.longitude
                    self.fpv_props.home_alt = self.fpv_props.filt_altitude
                    self.waiting_for_home = False

                # Compute the distance between home and current position
                self.fpv_props.distance = self.calc_distance(self.fpv_props.home_lon, self.fpv_props.home_lat,
                                                        self.packet.longitude, self.packet.latitude)

                # Compute the distance between home and current position
                self.fpv_props.distance = self.calc_distance(self.fpv_props.home_lon, self.fpv_props.home_lat,
                                                        self.packet.longitude, self.packet.latitude)

                # Compute the distance between home and current position
                self.fpv_props.dir_home = self.calc_bearing(self.packet.longitude, self.packet.latitude,
															self.fpv_props.home_lon, self.fpv_props.home_lat) - self.packet.bearing 

            # Calculate how many packets were lost since last pkt was received
            # Don't double-count corrupted packets: do this by recording pkt ID of last good packet, as well
            # as number of corrupted packets that we have thrown out since then.
            self.total_pkts_lost = self.total_pkts_lost + \
                                    max( ( (self.packet.pkt_cntr - (last_valid_pkt_cntr+1)) % 256 ) - self.contig_bad_crcs , 0 )
            self.contig_bad_crcs = 0

            # Print packet to console and to file
            self.print_log_packet()

            # Print computed characteristics to console
            self.print_fpv_props()
            
        # Else
        else:

            print "ERROR: bad CRC"
            self.total_pkts_lost += 1
            self.contig_bad_crcs += 1

        
    """
    Prints the current packet to the console, and saves it to a log file
    """
    def print_log_packet(self):

        self.print_log("Packet counter: " + str(self.packet.pkt_cntr))       
        self.print_log("12V battery: " + str(self.packet.bat_volt_12v) + "V\t" + str(self.packet.bat_curr_12v) + "A") 
        self.print_log("6V battery: " + str(self.packet.bat_volt_6v) + "V\t" + str(self.packet.bat_curr_6v) + "mA") 
        self.print_log("Spektrum RSSI: " + str((int)(self.packet.spektrum_rssi / 255.0 * 100.0)) + "%")
        self.print_log("Latitude: " + str(self.packet.latitude) + "deg")
        self.print_log("Longitude: " + str(self.packet.longitude) + "deg")
        self.print_log("Altitude: " + str(self.packet.altitude) + "m")
        self.print_log("Speed: " + str(self.packet.speed) + "km/h")
        self.print_log("Bearing: " + str(self.packet.bearing) + " deg")


    """
    Prints the given string to the console. If logging is enabled, writes it to file too.
    """
    def print_log(self, s):

        print s
        if(log_en):
            self.log_file.write(s + '\n')


    """
    Prints the computed FPV properties to the console
    """
    def print_fpv_props(self):

        print "Home co-ordinates: ", self.fpv_props.home_lat, ",", self.fpv_props.home_lon
        print "Home altitude: ", self.fpv_props.home_alt, "m"
        print "Distance: ", self.fpv_props.distance, "m"

        if(self.fpv_props.alt_valid):
            print "Filtered altitude: ", self.fpv_props.filt_altitude, "m"
        

    """
    Computes the distance between two geographical distance.
    The provided co-ordinates must be in decimal degrees.
    The returned result is in meters.
    """
    def calc_distance(self, lon1, lat1, lon2, lat2):

        # Convert decimal degrees to radians 
        lon1_r, lat1_r, lon2_r, lat2_r = map(radians, [lon1, lat1, lon2, lat2])

        # Haversine formula 
        dlon = lon2_r - lon1_r 
        dlat = lat2_r - lat1_r 
        a = sin(dlat/2)**2 + cos(lat1_r) * cos(lat2_r) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        dist_m = 6371 * 1000.0 * c
        return (int)(dist_m)


    """
    Computes the bearing from one point to another.
    The provided co-ordinates must be in decimal degrees.
    The returned result is in degrees.
    """
    def calc_bearing(self, lon1, lat1, lon2, lat2):

        # Convert decimal degrees to radians 
        lon1_r, lat1_r, lon2_r, lat2_r = map(radians, [lon1, lat1, lon2, lat2])

        bearing = atan2( sin(lon2_r-lon1_r) * cos(lat2_r),
                         cos(lat1_r) * sin(lat2_r) - sin(lat1_r) * cos(lat2_r) * cos(lon2_r-lon1_r))

        return (bearing % (2*pi)) / pi * 180.0 
            

    """
    Calling this function set's the home position.
    Note: the actual home position may only be set later, once the GPS has actually locked.
    """
    def set_home_pos(self):
        self.alt_filter = []
        self.fpv_props.home_alt = 0
        self.total_pkts_lost = 0
        self.fpv_props.alt_valid = False
        self.waiting_for_home = True


    """
    Closes the log file, if one exists. Called from the GUI.
    """
    def close_log(self):
        if(log_en):
            self.log_file.close()

