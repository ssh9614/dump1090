// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090.c: main program & miscellany
//
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it  
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your  
// option) any later version.  
//
// This file is distributed in the hope that it will be useful, but  
// WITHOUT ANY WARRANTY; without even the implied warranty of  
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License  
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// This file incorporates work covered by the following copyright and  
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "dump1090.h"

#include <rtl-sdr.h>

#include <stdarg.h>

static int verbose_device_search(char *s);

//
// ============================= Utility functions ==========================
//

static void log_with_timestamp(const char *format, ...) __attribute__((format (printf, 1, 2) ));

static void log_with_timestamp(const char *format, ...)
{
    char timebuf[128];
    char msg[1024];
    time_t now;
    struct tm local;
    va_list ap;

    now = time(NULL);
    localtime_r(&now, &local);
    strftime(timebuf, 128, "%c %Z", &local);
    timebuf[127] = 0;

    va_start(ap, format);
    vsnprintf(msg, 1024, format, ap);
    va_end(ap);
    msg[1023] = 0;

    fprintf(stderr, "%s  %s\n", timebuf, msg);
}

static void sigintHandler(int dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGINT, SIG_DFL);  // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
    log_with_timestamp("Caught SIGINT, shutting down..\n");
}

static void sigtermHandler(int dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGTERM, SIG_DFL); // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
    log_with_timestamp("Caught SIGTERM, shutting down..\n");
}

//
// =============================== Initialization ===========================
//
void modesInitConfig(void) {
    // Default everything to zero/NULL
    memset(&Modes, 0, sizeof(Modes));

    // Now initialise things that should not be 0/NULL to their defaults
    Modes.gain                    = MODES_MAX_GAIN;
    Modes.freq                    = MODES_DEFAULT_FREQ;
    Modes.ppm_error               = MODES_DEFAULT_PPM;
    Modes.net_heartbeat_interval  = MODES_NET_HEARTBEAT_INTERVAL;
    Modes.net_output_raw_ports    = strdup("30002");
}
//
//=========================================================================
//
void modesInit(void) {
    int i, q;

    pthread_mutex_init(&Modes.data_mutex,NULL);
    pthread_cond_init(&Modes.data_cond,NULL);

    Modes.sample_rate = 2400000.0;

    // Allocate the various buffers used by Modes
    Modes.trailing_samples = (MODES_PREAMBLE_US + MODES_LONG_MSG_BITS + 16) * 1e-6 * Modes.sample_rate;

    if ( ((Modes.maglut     = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL) ||
         ((Modes.log10lut   = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL) )
    {
        fprintf(stderr, "Out of memory allocating data buffer.\n");
        exit(1);
    }

    for (i = 0; i < MODES_MAG_BUFFERS; ++i) {
        if ( (Modes.mag_buffers[i].data = calloc(MODES_MAG_BUF_SAMPLES+Modes.trailing_samples, sizeof(uint16_t))) == NULL ) {
            fprintf(stderr, "Out of memory allocating magnitude buffer.\n");
            exit(1);
        }

        Modes.mag_buffers[i].length = 0;
        Modes.mag_buffers[i].dropped = 0;
        Modes.mag_buffers[i].sampleTimestamp = 0;
    }

    // Limit the maximum requested raw output size to less than one Ethernet Block 
    if (Modes.net_output_flush_size > (MODES_OUT_FLUSH_SIZE))
      {Modes.net_output_flush_size = MODES_OUT_FLUSH_SIZE;}
    if (Modes.net_output_flush_interval > (MODES_OUT_FLUSH_INTERVAL))
      {Modes.net_output_flush_interval = MODES_OUT_FLUSH_INTERVAL;}
    if (Modes.net_sndbuf_size > (MODES_NET_SNDBUF_MAX))
      {Modes.net_sndbuf_size = MODES_NET_SNDBUF_MAX;}

    // compute UC8 magnitude lookup table
    for (i = 0; i <= 255; i++) {
        for (q = 0; q <= 255; q++) {
            float fI, fQ, magsq;

            fI = (i - 127.5) / 127.5;
            fQ = (q - 127.5) / 127.5;
            magsq = fI * fI + fQ * fQ;
            if (magsq > 1)
                magsq = 1;

            Modes.maglut[le16toh((i*256)+q)] = (uint16_t) round(sqrtf(magsq) * 65535.0);
        }
    }

    // Prepare the log10 lookup table: 100log10(x)
    Modes.log10lut[0] = 0; // poorly defined..
    for (i = 1; i <= 65535; i++) {
        Modes.log10lut[i] = (uint16_t) round(100.0 * log10(i));
    }

    // Prepare error correction tables
    modesChecksumInit(1);
    icaoFilterInit();

    // Prepare sample conversion
    Modes.input_format = INPUT_UC8;
    Modes.converter_function = init_converter(Modes.input_format,
                                              &Modes.converter_state);
    if (!Modes.converter_function) {
      fprintf(stderr, "Can't initialize sample converter, giving up.\n");
      exit(1);
    }
}

static void convert_samples(void *iq,
                            uint16_t *mag,
                            unsigned nsamples,
                            double *power)
{
    Modes.converter_function(iq, mag, nsamples, Modes.converter_state, power);
}

//
// =============================== RTLSDR handling ==========================
//
int modesInitRTLSDR(void) {
    int j;
    int device_count, dev_index = 0;
    char vendor[256], product[256], serial[256];

    if (Modes.dev_name) {
        if ( (dev_index = verbose_device_search(Modes.dev_name)) < 0 )
            return -1;
    }

    device_count = rtlsdr_get_device_count();
    if (!device_count) {
        fprintf(stderr, "No supported RTLSDR devices found.\n");
        return -1;
    }

    fprintf(stderr, "Found %d device(s):\n", device_count);
    for (j = 0; j < device_count; j++) {
        if (rtlsdr_get_device_usb_strings(j, vendor, product, serial) != 0) {
            fprintf(stderr, "%d: unable to read device details\n", j);
        } else {
            fprintf(stderr, "%d: %s, %s, SN: %s %s\n", j, vendor, product, serial,
                    (j == dev_index) ? "(currently selected)" : "");
        }
    }

    if (rtlsdr_open(&Modes.dev, dev_index) < 0) {
        fprintf(stderr, "Error opening the RTLSDR device: %s\n",
            strerror(errno));
        return -1;
    }

    // Set gain, frequency, sample rate, and reset the device
    rtlsdr_set_tuner_gain_mode(Modes.dev,
        (Modes.gain == MODES_AUTO_GAIN) ? 0 : 1);
    if (Modes.gain != MODES_AUTO_GAIN) {
        int *gains;
        int numgains;

        numgains = rtlsdr_get_tuner_gains(Modes.dev, NULL);
        if (numgains <= 0) {
            fprintf(stderr, "Error getting tuner gains\n");
            return -1;
        }

        gains = malloc(numgains * sizeof(int));
        if (rtlsdr_get_tuner_gains(Modes.dev, gains) != numgains) {
            fprintf(stderr, "Error getting tuner gains\n");
            free(gains);
            return -1;
        }
        
        if (Modes.gain == MODES_MAX_GAIN) {
            int highest = -1;
            int i;

            for (i = 0; i < numgains; ++i) {
                if (gains[i] > highest)
                    highest = gains[i];
            }

            Modes.gain = highest;
            fprintf(stderr, "Max available gain is: %.2f dB\n", Modes.gain/10.0);
        } else {
            int closest = -1;
            int i;

            for (i = 0; i < numgains; ++i) {
                if (closest == -1 || abs(gains[i] - Modes.gain) < abs(closest - Modes.gain))
                    closest = gains[i];
            }

            if (closest != Modes.gain) {
                Modes.gain = closest;
                fprintf(stderr, "Closest available gain: %.2f dB\n", Modes.gain/10.0);
            }
        }

        free(gains);

        fprintf(stderr, "Setting gain to: %.2f dB\n", Modes.gain/10.0);
        if (rtlsdr_set_tuner_gain(Modes.dev, Modes.gain) < 0) {
            fprintf(stderr, "Error setting tuner gains\n");
            return -1;
        }
    } else {
        fprintf(stderr, "Using automatic gain control.\n");
    }
    rtlsdr_set_freq_correction(Modes.dev, Modes.ppm_error);
    if (Modes.enable_agc) rtlsdr_set_agc_mode(Modes.dev, 1);
    rtlsdr_set_center_freq(Modes.dev, Modes.freq);
    rtlsdr_set_sample_rate(Modes.dev, (unsigned)Modes.sample_rate);

    rtlsdr_reset_buffer(Modes.dev);
    fprintf(stderr, "Gain reported by device: %.2f dB\n",
        rtlsdr_get_tuner_gain(Modes.dev)/10.0);

    return 0;
}
//
//=========================================================================
//
// We use a thread reading data in background, while the main thread
// handles decoding and visualization of data to the user.
//
// The reading thread calls the RTLSDR API to read data asynchronously, and
// uses a callback to populate the data buffer.
//
// A Mutex is used to avoid races with the decoding thread.
//

void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx) {
    struct mag_buf *outbuf;
    struct mag_buf *lastbuf;
    uint32_t slen;
    unsigned next_free_buffer;
    unsigned free_bufs;
    unsigned block_duration;

    static int was_odd = 0; // paranoia!!
    static int dropping = 0;

    MODES_NOTUSED(ctx);

    // Lock the data buffer variables before accessing them
    pthread_mutex_lock(&Modes.data_mutex);
    if (Modes.exit) {
        rtlsdr_cancel_async(Modes.dev); // ask our caller to exit
    }

    next_free_buffer = (Modes.first_free_buffer + 1) % MODES_MAG_BUFFERS;
    outbuf = &Modes.mag_buffers[Modes.first_free_buffer];
    lastbuf = &Modes.mag_buffers[(Modes.first_free_buffer + MODES_MAG_BUFFERS - 1) % MODES_MAG_BUFFERS];
    free_bufs = (Modes.first_filled_buffer - next_free_buffer + MODES_MAG_BUFFERS) % MODES_MAG_BUFFERS;

    // Paranoia! Unlikely, but let's go for belt and suspenders here

    if (len != MODES_RTL_BUF_SIZE) {
        fprintf(stderr, "weirdness: rtlsdr gave us a block with an unusual size (got %u bytes, expected %u bytes)\n",
                (unsigned)len, (unsigned)MODES_RTL_BUF_SIZE);

        if (len > MODES_RTL_BUF_SIZE) {
            // wat?! Discard the start.
            unsigned discard = (len - MODES_RTL_BUF_SIZE + 1) / 2;
            outbuf->dropped += discard;
            buf += discard*2;
            len -= discard*2;
        }
    }

    if (was_odd) {
        // Drop a sample so we are in sync with I/Q samples again (hopefully)
        ++buf;
        --len;
        ++outbuf->dropped;
    }

    was_odd = (len & 1);
    slen = len/2;

    if (free_bufs == 0 || (dropping && free_bufs < MODES_MAG_BUFFERS/2)) {
        // FIFO is full. Drop this block.
        dropping = 1;
        outbuf->dropped += slen;
        pthread_mutex_unlock(&Modes.data_mutex);
        return;
    }

    dropping = 0;
    pthread_mutex_unlock(&Modes.data_mutex);

    // Compute the sample timestamp and system timestamp for the start of the block
    outbuf->sampleTimestamp = lastbuf->sampleTimestamp + 12e6 * (lastbuf->length + outbuf->dropped) / Modes.sample_rate;
    block_duration = 1e9 * slen / Modes.sample_rate;

    // Get the approx system time for the start of this block
    clock_gettime(CLOCK_REALTIME, &outbuf->sysTimestamp);
    outbuf->sysTimestamp.tv_nsec -= block_duration;
    normalize_timespec(&outbuf->sysTimestamp);

    // Copy trailing data from last block (or reset if not valid)
    if (outbuf->dropped == 0 && lastbuf->length >= Modes.trailing_samples) {
        memcpy(outbuf->data, lastbuf->data + lastbuf->length - Modes.trailing_samples, Modes.trailing_samples * sizeof(uint16_t));
    } else {
        memset(outbuf->data, 0, Modes.trailing_samples * sizeof(uint16_t));
    }

    // Convert the new data
    outbuf->length = slen;
    convert_samples(buf, &outbuf->data[Modes.trailing_samples], slen, &outbuf->total_power);

    // Push the new data to the demodulation thread
    pthread_mutex_lock(&Modes.data_mutex);

    Modes.mag_buffers[next_free_buffer].dropped = 0;
    Modes.mag_buffers[next_free_buffer].length = 0;  // just in case
    Modes.first_free_buffer = next_free_buffer;

    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
}
//
//=========================================================================
//
// We read data using a thread, so the main thread only handles decoding
// without caring about data acquisition
//

void *readerThreadEntryPoint(void *arg) {
    MODES_NOTUSED(arg);

    while (!Modes.exit) {
      rtlsdr_read_async(Modes.dev, rtlsdrCallback, NULL,
                        MODES_RTL_BUFFERS,
                        MODES_RTL_BUF_SIZE);

      if (!Modes.exit) {
        log_with_timestamp("Warning: lost the connection to the RTLSDR device.");
        rtlsdr_close(Modes.dev);
        Modes.dev = NULL;

        do {
          sleep(5);
          log_with_timestamp("Trying to reconnect to the RTLSDR device..");
        } while (!Modes.exit && modesInitRTLSDR() < 0);
      }
    }
    
    if (Modes.dev != NULL) {
      rtlsdr_close(Modes.dev);
      Modes.dev = NULL;
    }

    // Wake the main thread (if it's still waiting)
    pthread_mutex_lock(&Modes.data_mutex);
    Modes.exit = 1; // just in case
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);

    pthread_exit(NULL);
}

//
// ================================ Main ====================================
//
void showHelp(void) {
    printf(
"-----------------------------------------------------------------------------\n"
"| dump1090 ModeS Receiver     %45s |\n"
"-----------------------------------------------------------------------------\n"
"--device-index <index>   Select RTL device (default: 0)\n"
"--gain <db>              Set gain (default: max gain. Use -10 for auto-gain)\n"
"--enable-agc             Enable the Automatic Gain Control (default: off)\n"
"--freq <hz>              Set frequency (default: 1090 Mhz)\n"
"--net-bind-address <ip>  IP address to bind to (default: Any; Use 127.0.0.1 for private)\n"
"--net-ro-port <ports>    TCP raw output listen ports (default: 30002)\n"
"--net-ro-size <size>     TCP output minimum size (default: 0)\n"
"--net-ro-interval <rate> TCP output memory flush rate in seconds (default: 0)\n"
"--net-heartbeat <rate>   TCP heartbeat rate in seconds (default: 60 sec; 0 to disable)\n"
"--net-buffer <n>         TCP buffer size 64Kb * (2^n) (default: n=0, 64Kb)\n"
"--ppm <error>            Set receiver error in parts per million (default 0)\n"
"--help                   Show this help\n"
"\n",
MODES_DUMP1090_VARIANT " " MODES_DUMP1090_VERSION
    );
}

//
//=========================================================================
//
// This function is called a few times every second by main in order to
// perform tasks we need to do continuously, like accepting new clients
// from the net, and so forth
//
void backgroundTasks(void) {

    icaoFilterExpire();
    modesNetPeriodicWork();

}

//
//=========================================================================
//
int verbose_device_search(char *s)
{
	int i, device_count, device, offset;
	char *s2;
	char vendor[256], product[256], serial[256];
	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		return -1;
	}
	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
            if (rtlsdr_get_device_usb_strings(i, vendor, product, serial) != 0) {
                fprintf(stderr, "  %d:  unable to read device details\n", i);
            } else {
                fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
            }
	}
	fprintf(stderr, "\n");
	/* does string look like raw id number */
	device = (int)strtol(s, &s2, 0);
	if (s2[0] == '\0' && device >= 0 && device < device_count) {
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string exact match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strcmp(s, serial) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string prefix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strncmp(s, serial, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string suffix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		offset = strlen(serial) - strlen(s);
		if (offset < 0) {
			continue;}
		if (strncmp(s, serial+offset, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	fprintf(stderr, "No matching devices found.\n");
	return -1;
}
//
//=========================================================================
//
int main(int argc, char **argv) {
    int j;

    // Set sane defaults
    modesInitConfig();

    // signal handlers:
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigtermHandler);

    // Parse the command line options
    for (j = 1; j < argc; j++) {
        int more = j+1 < argc; // There are more arguments

        if (!strcmp(argv[j],"--device-index") && more) {
            Modes.dev_name = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--gain") && more) {
            Modes.gain = (int) (atof(argv[++j])*10); // Gain is in tens of DBs
        } else if (!strcmp(argv[j],"--enable-agc")) {
            Modes.enable_agc++;
        } else if (!strcmp(argv[j],"--freq") && more) {
            Modes.freq = (int) strtoll(argv[++j],NULL,10);
        } else if (!strcmp(argv[j],"--net-heartbeat") && more) {
            Modes.net_heartbeat_interval = (uint64_t)(1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--net-ro-size") && more) {
          Modes.net_output_flush_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-rate") && more) {
            Modes.net_output_flush_interval = 1000 * atoi(argv[++j]) / 15; // backwards compatibility
        } else if (!strcmp(argv[j],"--net-ro-interval") && more) {
            Modes.net_output_flush_interval = (uint64_t)(1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--net-ro-port") && more) {
            free(Modes.net_output_raw_ports);
            Modes.net_output_raw_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bind-address") && more) {
            free(Modes.net_bind_address);
            Modes.net_bind_address = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-buffer") && more) {
            Modes.net_sndbuf_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--help")) {
            showHelp();
            exit(0);
        } else if (!strcmp(argv[j],"--ppm") && more) {
            Modes.ppm_error = atoi(argv[++j]);
        } else {
            fprintf(stderr,
                "Unknown or not enough arguments for option '%s'.\n\n",
                argv[j]);
            showHelp();
            exit(1);
        }
    }

    // Initialization
    log_with_timestamp("%s %s starting up.", MODES_DUMP1090_VARIANT, MODES_DUMP1090_VERSION);
    modesInit();

    if (modesInitRTLSDR() < 0) {
      exit(1);
    }

    modesInitNet();

    int watchdogCounter = 10; // about 1 second

    // Create the thread that will read the data from the device.
    pthread_mutex_lock(&Modes.data_mutex);
    pthread_create(&Modes.reader_thread, NULL, readerThreadEntryPoint, NULL);

    while (Modes.exit == 0) {

      if (Modes.first_free_buffer == Modes.first_filled_buffer) {
        /* wait for more data.
         * we should be getting data every 50-60ms. wait for max 100ms before we give up and do some background work.
         * this is fairly aggressive as all our network I/O runs out of the background work!
         */

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100000000;
        normalize_timespec(&ts);

        pthread_cond_timedwait(&Modes.data_cond, &Modes.data_mutex, &ts); // This unlocks Modes.data_mutex, and waits for Modes.data_cond
      }

      // Modes.data_mutex is locked, and possibly we have data.

      if (Modes.first_free_buffer != Modes.first_filled_buffer) {
        // FIFO is not empty, process one buffer.

        struct mag_buf *buf;

        buf = &Modes.mag_buffers[Modes.first_filled_buffer];

        // Process data after releasing the lock, so that the capturing
        // thread can read data while we perform computationally expensive
        // stuff at the same time.
        pthread_mutex_unlock(&Modes.data_mutex);

        demodulate2400(buf);

        // Mark the buffer we just processed as completed.
        pthread_mutex_lock(&Modes.data_mutex);
        Modes.first_filled_buffer = (Modes.first_filled_buffer + 1) % MODES_MAG_BUFFERS;
        pthread_cond_signal(&Modes.data_cond);
        pthread_mutex_unlock(&Modes.data_mutex);
        watchdogCounter = 10;
      } else {
        // Nothing to process this time around.
        pthread_mutex_unlock(&Modes.data_mutex);
        if (--watchdogCounter <= 0) {
          log_with_timestamp("No data received from the dongle for a long time, it may have wedged");
          watchdogCounter = 600;
        }
      }

      backgroundTasks();

      pthread_mutex_lock(&Modes.data_mutex);
    }

    pthread_mutex_unlock(&Modes.data_mutex);

    log_with_timestamp("Waiting for receive thread termination");
    pthread_join(Modes.reader_thread,NULL);     // Wait on reader thread exit
    pthread_cond_destroy(&Modes.data_cond);     // Thread cleanup - only after the reader thread is dead!
    pthread_mutex_destroy(&Modes.data_mutex);

    cleanup_converter(Modes.converter_state);
    log_with_timestamp("Normal exit.");

    pthread_exit(0);
}
//
//=========================================================================
//
