% Figure out what devices are connected
dev = audiodevinfo();

for i = 1:length(dev.input)
    dev.input(i)
end

% Get the ID of the device that we want
dev_id = audiodevinfo(1, 'default (ALSA)');

% See what sample and bit rates it supports
check_modes = 0;
if check_modes == 1
    samp_rates = [8000, 11025, 22050, 32000, 44100, 48000, 96000, 192000, 352800];
    bit_depths = [8, 16, 24];

    for s = samp_rates
        for b = bit_depths
            supported = audiodevinfo(1, dev_id, s, b, 1);
            fprintf('Sample rate %d Hz, bit depth %d : %d\n', s, b, supported);
        end
    end
end

% Create the audio recorder with the necesary properties
f_samp = 48000;
bit_depth = 16;
recorder = audiorecorder(f_samp, bit_depth, 1);

% Record for 400ms
record_len = 0.4;
disp('Recording starting.')
recordblocking(recorder, record_len);
disp('End of Recording.');

% Play back the recording.
play(recorder);

% Store data in double-precision array.
samples = getaudiodata(recorder);
n_samps = length(samples);
time = 0 : 1/f_samp : record_len - 1/f_samp;

% Compute the FFT
spectrum = fftshift( fft(samples) );
delta_f = f_samp / n_samps;
freq = -f_samp/2 : delta_f : f_samp/2 - delta_f;

% Plot the waveforms
subplot(1,2,1); 
plot(time, samples)

subplot(1,2,2); 
plot(freq, abs(spectrum));

% Generate the matched filter to detect start of packet
symbol_len = 0.0015;
chirp_start_f = 500;
chirp_stop_f = 6000;
chirp_duration = symbol_len * 4;
matched_filt_t = 0 : 1/f_samp : chirp_duration - 1/f_samp;
matched_filt = chirp(matched_filt_t, chirp_start_f, chirp_duration - 1/f_samp, chirp_stop_f);
matched_filt = matched_filt .* linspace(1, 0.5, length(matched_filt_t));
matched_filt = fliplr(matched_filt);

% Apply the matched filter
filter_out = filter(matched_filt, 1, samples);

% Detect the peaks of the matched filter to find the start of each packet
filter_out_pwr = filter_out .^ 2;
figure();
plot(time, filter_out_pwr);

threshold = max(filter_out_pwr) * 0.75
peaks = filter_out_pwr;
peaks(peaks < threshold) = 0;

% Each peak is actually a set of multiple peaks, so we take the largest
% peak within a 10ms window
window_length = 0.010;
idx = 1;
while(idx <= length(time))
    
    % If this is the beginning of a peak
   if(peaks(idx) > 0)
      end_window = min(idx + f_samp * window_length, length(time));
      [max_val, max_idx] = max(peaks(idx:end_window))
      peaks(idx:end_window) = 0;
      peaks(idx + max_idx-1) = 1;
      
      idx = idx + f_samp * window_length;
      
   % Else not near a peak
   else
       idx = idx + 1;
   end
end

figure();
plot(time, peaks);

% Divide each packet up into symbol windows
pkt_len_bytes = 20;
pkt_len_symbols = pkt_len_bytes * 4;
packet_starts = find(peaks);
num_pkts = length(packet_starts) - 1;
symbol_samps = zeros([num_pkts, pkt_len_symbols, symbol_len * f_samp]);

for pkt = 1:num_pkts
    
    data_start = packet_starts(pkt);
    pkt_data = samples(data_start : data_start + pkt_len_symbols * symbol_len * f_samp - 1);
    symbol_samps(pkt, :, :) = reshape(pkt_data, [symbol_len * f_samp, pkt_len_symbols])';
    
end

% Take an FFT inside each window to determine the symbol in that window
% The FFT indices correspond to following frequencies:
% [0, 666, 1333, 2000, 2667, 3333, 4000, 4667, 5333, 6000, 6667, 7333, etc]
% The corresponding symbols are given in table below:
symbol_table = ones(f_samp/2*symbol_len) * -1;
symbol_table(1:11) = [-1, -1, -1, -1, 0, -1, 1, -1, 2, -1, 3];
symbols = zeros([num_pkts, pkt_len_symbols]);

for pkt = 1:num_pkts
    for sym = 1:pkt_len_symbols
        
        % Compute the frequency of each symbol
        symbol_fft = fft(symbol_samps(pkt, sym, :));
        [symbol_freq_mag, symbol_freq_idx] = max( abs(symbol_fft(1:length(symbol_fft)/2)) );
         
        symbols(pkt, sym) = symbol_table(symbol_freq_idx);
    end
end

figure();
plot(symbols(1,:));

% Generate the CRC detector
crc_detector = crc.detector('Polynomial', '0x8005', 'InitialState', '0x0000');
num_crc_errors = 0;

% Convert the symbols to bits and recover the original packet
% TODO: check for bad data (i.e. -1s)
for pkt = 1:num_pkts

    pkt_symbols = reshape(symbols(pkt,:), [4, pkt_len_bytes])';
    pkt_bytes = zeros(pkt_len_bytes, 1);
    
    for byte = 1:pkt_len_bytes
        pkt_bytes(byte) = pkt_symbols(byte, 4) * 64 + pkt_symbols(byte, 3) * 16 + pkt_symbols(byte, 2) * 4 + pkt_symbols(byte, 1) * 1;
    end
    
    % Check if CRC byte is correct
    pkt_bits = reshape(de2bi(pkt_bytes, 8, 'left-msb')', pkt_len_bytes * 8, 1);
    [out_data crc_error] = detect(crc_detector, pkt_bits);

    disp(pkt_bytes(1:end-2));
    %disp(char(pkt_bytes(1:end-2)));
    fprintf(1, 'Error state = %d\n', crc_error);
    num_crc_errors = num_crc_errors + crc_error;
    
    % Decode the packet
    pkt_counter = pkt_bytes(1)
    bat_volt_12v = (pkt_bytes(3)*256 + pkt_bytes(2)) * 0.001
    bat_curr_12v = pkt_bytes(4) * 0.1
    bat_volt_6v = pkt_bytes(5) * 0.01 + 5
    bat_curr_6v = pkt_bytes(6) * 0.01
    rcvr_rssi = pkt_bytes(7)
    latitude_bytes = [uint8(pkt_bytes(8:10)); uint8(0)];
    latitude = double(typecast(latitude_bytes, 'int32')) / 100000
    longitude = double(typecast(uint8(pkt_bytes(11:14)), 'int32')) / 100000
    altitude = pkt_bytes(15)
    velocity = pkt_bytes(16)
    bearing = (pkt_bytes(18)*256 + pkt_bytes(17)) * 0.01

end

num_crc_errors
