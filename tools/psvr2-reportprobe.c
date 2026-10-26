// Reads the PS VR2 headset's "get" reports (vendor control IN, request 1, as Monado's
// get_psvr2_control: 0x81 firmware, 0x8f display calibration) over a range of report ids and
// subcommands, to find what else it keeps, such as its cameras' calibration. Only reads.
//
//   cc -O2 tools/psvr2-reportprobe.c -I$(brew --prefix)/include -L$(brew --prefix)/lib -lusb-1.0 -o build/psvr2-reportprobe
//   build/psvr2-reportprobe <out dir> [first last]     (report ids, default 0x80 0xff)
//
// Writes r<id>_<subcmd>.bin for each that answers. Nothing else may have the headset open.
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <out dir> [first last]\n", argv[0]); return 1; }
    int first = argc > 3 ? (int)strtol(argv[2], NULL, 0) : 0x80, last = argc > 3 ? (int)strtol(argv[3], NULL, 0) : 0xff;
    libusb_init(NULL);
    libusb_device_handle *dev = libusb_open_device_with_vid_pid(NULL, 0x054C, 0x0CDE);
    if (!dev) { fprintf(stderr, "no headset (or something else has it)\n"); return 1; }
    for (int id = first; id <= last; id++)
        for (int sub = 0; sub < (getenv("SUBS") ? atoi(getenv("SUBS")) : 4); sub++) {
            unsigned char pkt[512] = {0};
            pkt[0] = id & 0xff; pkt[1] = id >> 8; pkt[2] = sub; pkt[4] = 0xf8; pkt[5] = 1;   // len 504
            int r = libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT | LIBUSB_ENDPOINT_IN,
                                            0x1, id, 0, pkt, sizeof pkt, 300);
            if (r <= 8) continue;
            int nz = 0;
            for (int i = 8; i < r; i++) nz += pkt[i] != 0;
            if (!nz) continue;
            char path[1024];
            snprintf(path, sizeof path, "%s/r%02x_%d.bin", argv[1], id, sub);
            FILE *f = fopen(path, "wb");
            if (f) { fwrite(pkt, 1, r, f); fclose(f); }
            printf("report 0x%02x subcmd %d: %d bytes, %d non-zero, header len %u\n", id, sub, r, nz, pkt[4] | pkt[5] << 8);
        }
    libusb_close(dev);
    return 0;
}
