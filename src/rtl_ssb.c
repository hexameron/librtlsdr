/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * rtl-ssb, cut down version of rtl_fm, with fixed usb mode and 8 kHz output
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2014 by John Greb
 *
 * UDP modifications: Original Code by Olgierd Pilarczyk
 * Extended by Frederik Granna <rtlsdr@granna.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * written because people could not do real time
 * FM demod on Atom hardware with GNU radio
 * based on rtl_sdr.c and rtl_tcp.c
 *
 * lots of locks, but that is okay
 *
 * todo:
 *       watchdog to reset bad dongle
 *       fix oversampling
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#ifdef _MSC_VER
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <pthread.h>
#include <libusb.h>

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define DEFAULT_SAMPLE_RATE		24000
#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define AUTO_GAIN			-100

static pthread_t socket_thread;
static void optimal_settings(int freq, int rate);
static int digiboost = 1;

static volatile int do_exit = 0;

struct dongle_state
{
	int      exit_flag;
	pthread_t thread;
	rtlsdr_dev_t *dev;
	int      dev_index;
	uint32_t freq;
	uint32_t rate;
	int      gain;
	uint16_t buf16[MAXIMUM_BUF_LENGTH];
	uint32_t buf_len;
	int      ppm_error;
	int      offset_tuning;
	int      direct_sampling;
	int      mute;
	struct demod_state *demod_target;
};

struct demod_state
{
	int      exit_flag;
	pthread_t thread;
	int16_t  lowpassed[MAXIMUM_BUF_LENGTH];
	int      lp_len;
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int      result_len;
	int      rate_in;
	int      rate_out;
	int      prev_index;
	int      downsample;    /* min 1, max 256 */
	int      post_downsample;
	int      output_scale;
	int      downsample_passes;
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	struct output_state *output_target;
};

struct output_state
{
	int      exit_flag;
	pthread_t thread;
	FILE     *file;
	char     *filename;
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int      result_len;
	int      rate;
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
};

struct controller_state
{
	int      exit_flag;
	uint32_t freqs[2];
	int      freq_len;
	int      freq_now;
};

// multiple of these, eventually
struct dongle_state dongle;
struct demod_state demod;
struct output_state output;
struct controller_state controller;

void usage(void)
{
	fprintf(stderr,
		"rtl_ssb, a simple narrow band ssb demodulator for RTL2832 based DVB-T receivers\n\n"
		"Use:\trtl_ssb -f freq [-options] [filename]\n"
		"\t-f frequency_to_tune_to [Hz]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-g tuner_gain (default: automatic)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\tfilename ('-' means stdout)\n"
		"\t    omitting the filename also uses stdout\n\n"
		"\n"
		"\trtl_ssb -f 434500000\n" );
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dongle.dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dongle.dev);
}
#endif

/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)

/* quarter samplerate of interleaved I or Q samples
        scales x6, which is safe with default settings */
void quartersample(int16_t *data, int length)
{
	int i;
	for (i = 0; (i*4)<(length - 6); i+=2)
		data[i] = ( data[i*4] + 2*(data[i*4 +2] + data[i*4 +4])+ data[i*4 + 6] );
}

/* This is not single sideband. The processing needed to remove the extra sideband would often 
 *	add more noise than the signal it removes, for narrow band use.
 */
void ssb_demod(struct demod_state *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;

	// downsample 2^6, scale up x216
	for (i=0; i<3; i++){
		quartersample(fm->lowpassed, fm->lp_len -1);
		quartersample(fm->lowpassed+1, fm->lp_len-1);
		fm->lp_len >>= 2;
	}

	/* Combine I/Q: it would be better to output stereo samples */
	for (i = 0; i < fm->lp_len; i += 2) {
		pcm = digiboost * (lp[i] + lp[i+1]);
		/* better to output I/Q as stereo samples */
		r[i/2] = pcm >> 1;
	}
	fm->result_len = fm->lp_len/2;
}

/*udp*/
static unsigned int chars_to_int(unsigned char* buf) {

	int i;
	unsigned int val = 0;

	for(i=1; i<5; i++) {
		val = val | ((buf[i]) << ((i-1)*8));
	}

	return val;
}

static void *socket_thread_fn(void *arg) {
	struct controller_state *fm = arg;
	int port = 6020;
  int r, n;
  int sockfd, newsockfd, portno;
  int new_freq, new_gain, agc_mode;
  socklen_t clilen;
  unsigned char buffer[5];
  struct sockaddr_in serv_addr, cli_addr;

	sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sockfd < 0) {
  		perror("ERROR opening socket");
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(port);

	if (bind(sockfd, (struct sockaddr *) &serv_addr,  sizeof(serv_addr)) < 0) {
		perror("ERROR on binding");
	}

	bzero(buffer,5);

	fprintf (stderr, "Socket thread started: Tuning enabled on UDP/%d \n", port);

	while((n = read(sockfd,buffer,5)) != 0) {

		if(buffer[0] == 0) {
			new_freq = chars_to_int(buffer);
			optimal_settings(new_freq, 0);
			//verbose_set_frequency(dongle.dev, dongle.freq);
			r = rtlsdr_set_center_freq(dongle.dev, dongle.freq);
			if (r < 0)
				fprintf(stderr, "WARNING: Failed to set center freq.\n");
		}

		if(buffer[0] == 1) {
			// exit
			int type = chars_to_int(buffer);
			do_exit = 1;
			break;
		}

		if (buffer[0] == 2) {
			digiboost = 1 + (7 & chars_to_int(buffer));
			fprintf (stderr, "Digital boost: %dx gain.\n", digiboost);
		}

		if (buffer[0] == 3) {
			new_gain = chars_to_int(buffer);
			if (new_gain == AUTO_GAIN) {
				r = verbose_auto_gain(dongle.dev);
			} else {
				new_gain = nearest_gain(dongle.dev, new_gain);
				r = verbose_gain_set(dongle.dev, new_gain);
			}
		}

		if (buffer[0] == 4) {
			agc_mode = chars_to_int(buffer);
			if (agc_mode == 0 || agc_mode == 1) {
				fprintf(stderr, "Setting AGC to %d\n", agc_mode);
				rtlsdr_set_agc_mode(dongle.dev, agc_mode);
			} else {
				fprintf(stderr, "Failed to set AGC to %d\n", agc_mode);
			}
		}
	}

	close(sockfd);
	return 0;
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	int i;
	struct dongle_state *s = ctx;
	struct demod_state *d = s->demod_target;

	if (do_exit) {
		return;}
	if (!ctx) {
		return;}
	if (s->mute) {
		for (i=0; i<s->mute; i++) {
			buf[i] = 127;}
		s->mute = 0;
	}
	for (i=0; i<(int)len; i++) {
		s->buf16[i] = (int16_t)buf[i] - 127;}
	pthread_rwlock_wrlock(&d->rw);
	memcpy(d->lowpassed, s->buf16, 2*len);
	d->lp_len = len;
	pthread_rwlock_unlock(&d->rw);
	safe_cond_signal(&d->ready, &d->ready_m);
}

static void *dongle_thread_fn(void *arg)
{
	struct dongle_state *s = arg;
	rtlsdr_read_async(s->dev, rtlsdr_callback, s, 0, s->buf_len);
	return 0;
}

static void *demod_thread_fn(void *arg)
{
	struct demod_state *d = arg;
	struct output_state *o = d->output_target;
	while (!do_exit) {
		safe_cond_wait(&d->ready, &d->ready_m);
		pthread_rwlock_wrlock(&d->rw);
		ssb_demod(d);
		pthread_rwlock_unlock(&d->rw);

		if (d->exit_flag) {
			do_exit = 1;
		}
		pthread_rwlock_wrlock(&o->rw);
		memcpy(o->result, d->result, 2*d->result_len);
		o->result_len = d->result_len;
		pthread_rwlock_unlock(&o->rw);
		safe_cond_signal(&o->ready, &o->ready_m);
	}
	return 0;
}

static void *output_thread_fn(void *arg)
{
	struct output_state *s = arg;
	while (!do_exit) {
		// use timedwait and pad out under runs
		safe_cond_wait(&s->ready, &s->ready_m);
		pthread_rwlock_rdlock(&s->rw);
		fwrite(s->result, 2, s->result_len, s->file);
		pthread_rwlock_unlock(&s->rw);
	}
	return 0;
}

static void optimal_settings(int freq, int rate)
{
	// giant ball of hacks
	// seems unable to do a single pass, 2:1
	int capture_freq, capture_rate;
	struct dongle_state *d = &dongle;
	struct demod_state *dm = &demod;
	struct controller_state *cs = &controller;
	dm->downsample = (1000000 / dm->rate_in) + 1;
	if (dm->downsample_passes) {
		dm->downsample_passes = (int)log2(dm->downsample) + 1;
		dm->downsample = 1 << dm->downsample_passes;
	}
	capture_freq = freq;
	capture_rate = dm->downsample * dm->rate_in;
	if (!d->offset_tuning) {
		capture_freq = freq + capture_rate/4;}
	d->freq = (uint32_t)capture_freq;
	d->rate = (uint32_t)capture_rate;
}

/* UDP handles retuning, only need to call this once */
void controller_start(struct controller_state *s)
{
	int i;

	optimal_settings(s->freqs[0], demod.rate_in);
	if (dongle.direct_sampling) {
		verbose_direct_sampling(dongle.dev, 1);}
	if (dongle.offset_tuning) {
		verbose_offset_tuning(dongle.dev);}

	/* Set the frequency */
	verbose_set_frequency(dongle.dev, dongle.freq);
	fprintf(stderr, "Oversampling output by: 64x.\n");
	fprintf(stderr, "Buffer size: %0.2fms\n",
		1000 * 0.5 * (float)DEFAULT_BUF_LENGTH / (float)dongle.rate);

	/* Set the sample rate */
	verbose_set_sample_rate(dongle.dev, dongle.rate);
	fprintf(stderr, "Output at %u Hz.\n", DEFAULT_SAMPLE_RATE);
}

void dongle_init(struct dongle_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	s->gain = AUTO_GAIN; // tenths of a dB
	s->mute = 0;
	s->direct_sampling = 0;
	s->offset_tuning = 0;
	s->demod_target = &demod;
}

void demod_init(struct demod_state *s)
{
	s->rate_in = 8 * DEFAULT_SAMPLE_RATE;
	s->rate_out = 8 * DEFAULT_SAMPLE_RATE;
	s->downsample_passes = 1;
	s->prev_index = 0;
	s->post_downsample = 1;  // once this works, default = 4
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
	s->output_target = &output;
}

void demod_cleanup(struct demod_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void output_init(struct output_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
}

void output_cleanup(struct output_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void controller_init(struct controller_state *s)
{
	s->freqs[0] = 100000000;
	s->freq_len = 0;
}

void sanity_checks(void)
{
	if (controller.freq_len == 0) {
		fprintf(stderr, "Please specify a frequency.\n");
		exit(1);
	}
}

/* 16KHz very long wav header */
void writewavheader(FILE *outfile)
{
	char wavhead[] = {
	0x52,0x49, 0x46,0x46, 0x64,0x19, 0xff,0x7f, 0x57,0x41, 0x56,0x45, 0x66,0x6d, 0x74,0x20,
	0x10,0x00, 0x00,0x00, 0x01,0x00, 0x01,0x00, 0xc0,0x5d, 0x00,0x00, 0x80,0xbb, 0x00,0x00,
	0x02,0x00, 0x10,0x00, 0x64,0x61, 0x74,0x61, 0x40,0x19, 0xff,0x7f, 0x00,0x00, 0x00,0x00
	};
	fwrite(wavhead, 2, sizeof(wavhead), outfile);
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	int r, opt;
	int dev_given = 0;
	int custom_ppm = 0;
	dongle_init(&dongle);
	demod_init(&demod);
	output_init(&output);
	controller_init(&controller);

	while ((opt = getopt(argc, argv, "d:f:g:p:h")) != -1) {
		switch (opt) {
		case 'd':
			dongle.dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			controller.freqs[0] = (uint32_t)atofs(optarg);
			controller.freq_len = 1;
			break;
		case 'g':
			dongle.gain = (int)(atof(optarg) * 10);
			break;
		case 'p':
			dongle.ppm_error = atoi(optarg);
			custom_ppm = 1;
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}

	/* quadruple sample_rate to limit to Δθ to ±π/2 */
	demod.rate_in *= demod.post_downsample;

	if (!output.rate) {
		output.rate = demod.rate_out;}

	sanity_checks();

	if (argc <= optind) {
		output.filename = "-";
	} else {
		output.filename = argv[optind];
	}

	if (!dev_given) {
		dongle.dev_index = verbose_device_search("0");
	}

	if (dongle.dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dongle.dev, (uint32_t)dongle.dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dongle.dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	/* Set the tuner gain */
	if (dongle.gain == AUTO_GAIN) {
		verbose_auto_gain(dongle.dev);
	} else {
		dongle.gain = nearest_gain(dongle.dev, dongle.gain);
		verbose_gain_set(dongle.dev, dongle.gain);
	}

	verbose_ppm_set(dongle.dev, dongle.ppm_error);

	if (strcmp(output.filename, "-") == 0) { /* Write samples to stdout */
		output.file = stdout;
#ifdef _WIN32
		_setmode(_fileno(output.file), _O_BINARY);
#endif
	} else {
		output.file = fopen(output.filename, "wb");
		if (!output.file) {
			fprintf(stderr, "Failed to open %s\n", output.filename);
			exit(1);
		}
	}
	writewavheader(output.file);

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dongle.dev);

	controller_start(&controller);
	usleep(100000);
	pthread_create(&output.thread, NULL, output_thread_fn, (void *)(&output));
	pthread_create(&demod.thread, NULL, demod_thread_fn, (void *)(&demod));
	pthread_create(&dongle.thread, NULL, dongle_thread_fn, (void *)(&dongle));
	pthread_create(&socket_thread, NULL, socket_thread_fn, (void *)(&controller));

	while (!do_exit) {
		usleep(100000);
	}

	fprintf(stderr, "\nRTL_FM: User cancel, exiting...\n");

	pthread_cancel(socket_thread);
	rtlsdr_cancel_async(dongle.dev);
	pthread_join(dongle.thread, NULL);
	safe_cond_signal(&demod.ready, &demod.ready_m);
	pthread_join(demod.thread, NULL);
	safe_cond_signal(&output.ready, &output.ready_m);
	pthread_join(output.thread, NULL);

	demod_cleanup(&demod);
	output_cleanup(&output);

	if (output.file != stdout) {
		fclose(output.file);}

	rtlsdr_close(dongle.dev);
	return r >= 0 ? r : -r;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
