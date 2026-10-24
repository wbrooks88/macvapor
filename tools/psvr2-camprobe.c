// Records what the PS VR2 headset sends in each camera mode, and from its LED detector stream, to
// find the upper cameras. Camera modes from Monado's psvr2_protocol.h (BSL-1.0).
//
//   cc -O2 tools/psvr2-camprobe.c -I$(brew --prefix)/include -L$(brew --prefix)/lib -lusb-1.0 -o build/psvr2-camprobe
//   build/psvr2-camprobe <out dir> [mode ...]     (modes in hex; default 4 b c 10)
//
// Writes mode<M>_<bytes>_<n>.bin (each packet as it came, header and all) and ld_<M>_<n>.bin, and
// prints how many packets of each size arrived. Nothing else may have the headset open.
#include <libusb-1.0/libusb.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define VID 0x054C
#define PID 0x0CDE
#define CAMERA_INTERFACE 6
#define CAMERA_ENDPOINT 0x87
#define LD_INTERFACE 8
#define LD_ENDPOINT 0x89

static libusb_device_handle *dev;
static const char *out;
static volatile int running, cur_mode;

static double now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int set_mode(int mode) {
    unsigned char pkt[8 + 8] = {0};
    pkt[0] = 0x0B;                  // report id
    pkt[2] = 1;                     // subcmd
    pkt[4] = 8;                     // length
    pkt[8] = 1;
    pkt[12] = (unsigned char)mode;
    return libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT, 0x9, 0x0B, 0, pkt,
                                   sizeof pkt, 1000);
}

static void save(const char *name, const unsigned char *b, int n) {
    char path[1024];
    snprintf(path, sizeof path, "%s/%s", out, name);
    FILE *f = fopen(path, "wb");
    if (f) { fwrite(b, 1, n, f); fclose(f); }
}

static void *ld_thread(void *arg) {
    (void)arg;
    static unsigned char buf[65536];
    int saved = 0, last_mode = -1, count = 0, bytes = 0;
    while (running) {
        int got = 0, r = libusb_bulk_transfer(dev, LD_ENDPOINT, buf, sizeof buf, &got, 200);
        if (cur_mode != last_mode) {
            if (last_mode >= 0) printf("  LED detector, mode %x: %d packets, %d bytes\n", last_mode, count, bytes);
            last_mode = cur_mode; saved = 0; count = 0; bytes = 0;
        }
        if (r == 0 && got > 0) {
            count++; bytes += got;
            if (saved < 8) {
                char name[64];
                snprintf(name, sizeof name, "ld_%x_%d.bin", cur_mode, saved++);
                save(name, buf, got);
            }
        }
    }
    printf("  LED detector, mode %x: %d packets, %d bytes\n", last_mode, count, bytes);
    return NULL;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <out dir> [mode ...]\n", argv[0]); return 1; }
    out = argv[1];
    int modes[32], nm = 0;
    for (int i = 2; i < argc && nm < 32; i++) modes[nm++] = (int)strtol(argv[i], NULL, 16);
    if (!nm) { modes[0] = 4; modes[1] = 0xb; modes[2] = 0xc; modes[3] = 0x10; nm = 4; }
    libusb_init(NULL);
    dev = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!dev) { fprintf(stderr, "headset not found on USB\n"); return 1; }
    libusb_set_auto_detach_kernel_driver(dev, 1);
    int r1 = libusb_claim_interface(dev, CAMERA_INTERFACE), r2 = libusb_claim_interface(dev, LD_INTERFACE);
    if (r1 < 0) { fprintf(stderr, "camera interface: %s\n", libusb_error_name(r1)); return 1; }
    if (r2 < 0) fprintf(stderr, "LED detector interface: %s\n", libusb_error_name(r2));
    running = 1;
    cur_mode = modes[0];
    pthread_t ld;
    if (r2 == 0) pthread_create(&ld, NULL, ld_thread, NULL);
    unsigned char *buf = malloc(4 << 20);
    for (int m = 0; m < nm; m++) {
        int mode = modes[m];
        cur_mode = mode;
        int sizes[16] = {0}, counts[16] = {0}, saved[16] = {0}, ns = 0;
        printf("mode %x: set %d\n", mode, set_mode(mode));
        double start = now(), last_set = start;
        while (now() - start < 4) {
            if (now() - last_set > 1) { set_mode(mode); last_set = now(); }
            int got = 0, r = libusb_bulk_transfer(dev, CAMERA_ENDPOINT, buf, 4 << 20, &got, 300);
            if (r != 0 && r != LIBUSB_ERROR_TIMEOUT) { printf("  read: %s\n", libusb_error_name(r)); continue; }
            if (got <= 0) continue;
            int k;
            for (k = 0; k < ns && sizes[k] != got; k++) {}
            if (k == ns && ns < 16) sizes[ns++] = got;
            if (k < 16) {
                counts[k]++;
                if (saved[k] < 6) {
                    char name[64];
                    snprintf(name, sizeof name, "mode%x_%d_%d.bin", mode, got, saved[k]++);
                    save(name, buf, got);
                }
            }
        }
        for (int k = 0; k < ns; k++) printf("  %d packets of %d bytes (%.1f/s)\n", counts[k], sizes[k], counts[k] / 4.0);
    }
    set_mode(0);
    running = 0;
    if (r2 == 0) pthread_join(ld, NULL);
    libusb_release_interface(dev, CAMERA_INTERFACE);
    if (r2 == 0) libusb_release_interface(dev, LD_INTERFACE);
    libusb_close(dev);
    return 0;
}
