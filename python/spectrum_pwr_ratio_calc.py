            # Another way to detect if we have a signal is to compare power < 2000Hz to power above 2000Hz
            # If we have signal, we should not be transmitting below 2000Hz
            # Ratio = 0.6 for noise, 8 for signal. Make threshold around 2
            fft_len = 8192
            signal_fft = abs(numpy.fft.rfft(self.buf[0:self.buf_cnt]))
            fft_freq = numpy.linspace(0, self.SAMP_RATE/2, len(signal_fft))
            fft_delta_f = self.SAMP_RATE/2.0 / len(signal_fft)
            plt.figure()
            plt.plot(fft_freq, signal_fft)
            plt.show()
            fft_ratio = (sum(signal_fft[(int)(2617/fft_delta_f):(int)(2717/fft_delta_f)]) + \
                        sum(signal_fft[(int)(3950/fft_delta_f):(int)(4050/fft_delta_f)]) + \
                        sum(signal_fft[(int)(5283/fft_delta_f):(int)(5383/fft_delta_f)]) + \
                        sum(signal_fft[(int)(6617/fft_delta_f):(int)(6717/fft_delta_f)]) ) / \
                        sum(signal_fft[0:(int)(400/fft_delta_f)])
            #print "df=", fft_delta_f, " idx=", 500/fft_delta_f, " fft len=", len(signal_fft)
            print "FFT ratio = ", fft_ratio


