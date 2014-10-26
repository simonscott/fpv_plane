ptx = 200e-3;   % 200mw
f = 1.28e9;
rcvr_sens = -85; % -85 dBm is typical, high-end devices are -93dBm
gain_tx = 4;    % dBi (monopole), 5 is theoretical maximum
gain_rx = 8;    % dBi (patch)
bw = 17e6;      % bandwidth of SAW filter on receiver

R = 20:1:5000;
lambda = 3e8/f;
fspl = ((4*pi*R) ./ lambda).^2;

p_tx_db = 10*log10(ptx);	% this was log, not log, 10 previously
fspl_db = 10*log10(fspl);   % this value is negative

p_rx_db = p_tx_db + gain_tx + gain_rx - fspl_db;
p_rx = 10.^(p_rx_db ./ 10);
p_rx_dbm = 10*log10(p_rx .* 1000);

plot(R, p_rx_dbm);