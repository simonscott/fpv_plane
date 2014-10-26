function generate_sin_table()

% Parameters
filename = '/media/Data/Programming/arduino/teensy_tones/sine_9b_256samps.txt';
bit_depth = 9;
num_samps = 256;

% Generate sine wave
t = 0:1:num_samps-1;
sine = sin(2*pi*(1/num_samps)*t);

% Quantize
sine = sine * (2 ^ (bit_depth-1) - 1) * 0.8 + 2 ^ (bit_depth-1) - 1;
sine = int32(round(sine));

% Plot waveform
plot(sine);

% Save to file
csvwrite(filename, sine);

end