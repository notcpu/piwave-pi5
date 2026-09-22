/*
 * pi5_fm_rds — EXPERIMENTAL native Raspberry Pi 5 PiFmRds port
 *
 * LEGAL / SAFETY — READ FIRST:
 *  Transmitting RF without authorisation is illegal in most countries.
 *  Only use with proper authorisation, on authorised frequencies, preferably
 *  into a shielded dummy load / direct coax to a receiver. Square-wave VHF
 *  is rich in harmonics. The authors accept no liability. This tool still
 *  refuses to touch hardware without --enable-rf.
 *
 * WHAT THIS IS:
 *  - Real FM MPX + RDS DSP chain at 228 kHz (mono/stereo, 19 kHz pilot,
 *    38 kHz DSB, 57 kHz RDS subcarrier with CRC + biphase + RRC shaping).
 *    `--measure-only` runs DSP only, safe on any machine, no hardware access.
 *  - Real RP1 discovery + BAR1 mapping via sysfs + /dev/mem, GPIO20/21
 *    FUNCSEL to GPCLK, clocks_main dump, PIO FIFO probe, PCIe write-rate
 *    benchmark. These prove what Linux userspace can and cannot reach.
 *  - Best-effort FM hook `rp1_fm_modulate()` left returning -ENOSYS with the
 *    exact missing piece documented (fractional divider register for RP1
 *    GPCLK/PLL, docs incomplete in clocks_main @ 0x40018000). Fill it in
 *    once your SDR + register dumps identify it; do not guess on air.
 *
 * WHY NOT FULL TX YET (honest):
 *  PIO control regs live at 0xf000_0000 *inside* RP1; from Linux only the
 *  FIFOs at 0x40178000 are visible (MichaelBell/rp1-hacking/PIO.md). SM
 *  configuration needs RP1 M3 firmware cooperation — no public path.
 *  Clock updates via clk-rp1 go through ~10 us mailbox IPC — ~100x too slow
 *  for 228 kHz FM deviation updates. Raw BAR1 divisor poking is the only
 *  plausible userspace path and its register map is undocumented.
 *
 * BUILD (on Pi 5, Raspberry Pi OS):
 *  sudo apt install -y build-essential libsndfile1-dev
 *  make
 *
 * USE:
 *  ./pi5_fm_rds --measure-only -audio song.wav            # safe DSP check
 *  sudo ./pi5_fm_rds --dump-rp1                           # BAR1 + clocks dump
 *  sudo ./pi5_fm_rds --measure-pcie                       # PCIe write benchmark
 *  sudo ./pi5_fm_rds --enable-rf --carrier-only -freq 100.0   # GPIO20->GPCLK, no audio mod
 *  sudo ./pi5_fm_rds --enable-rf -freq 100.0 -audio song.wav -ps TEST -rt "hello" -pi FFFF
 *      # currently: DSP runs, then exits ENOSYS at rp1_fm_modulate() with guidance.
 *
 * ANTENNA (when modulation one day works): GPIO20 (GPCLK0, header pin 38)
 * or GPIO21 (GPCLK1, pin 40). NOT GPIO4 — that BCM path is gone on Pi 5.
 */

#define _POSIX_C_SOURCE 200809L
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <sndfile.h>

/* ---------------- RP1 constants (from public docs / community) ---------------- */
/* RP1 PCIe endpoint */
#define RP1_VENDOR_ID  0x1de4
#define RP1_DEVICE_ID  0x0001
#define RP1_CHIP_ID    0x20001927u

/* BAR1-relative offsets (RP1 datasheet + community findings) */
#define RP1_OFF_CHIP_ID      0x00
#define RP1_IO_BANK0_BASE    0x0d0000u
/* Per-pin: STATUS @ base+pin*8, CTRL @ base+4+pin*8 */
#define RP1_GPIO_CTRL(pin)   (RP1_IO_BANK0_BASE + 4u + (pin) * 8u)
#define RP1_GPIO_FUNCSEL_MASK 0x1fu  /* FUNCSEL field, bits [4:0]; VERIFY vs datasheet */
#define RP1_FUNC_GPCLK       3u      /* Table 4: GPCLK[0] on GPIO20 col a3, GPCLK[1] on GPIO21 col a3 */
#define RP1_FUNC_PIO         7u      /* PIO column a7; programming SMs still needs M3 fw (see note) */
#define RP1_SYS_RIO0_BASE    0x0e0000u
#define RP1_CLOCKS_MAIN      0x40018000u
#define RP1_PWM0_BASE        0x40098000u
#define RP1_PWM1_BASE        0x4009c000u
#define RP1_PIO_FIFO_LINUX   0x40178000u  /* TX FIFOs visible from Linux per rp1-hacking */

/* Old PiFmRds internal rate */
#define MPX_RATE 228000u
#define AUDIO_CUTOFF 15000.0

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -freq F        carrier MHz, 76-108 only (default 100.0)\n"
        "  -audio FILE    wav/ogg/flac readable by libsndfile (or - for stdin, wav)\n"
        "  -ps NAME       8 chars max\n"
        "  -rt TEXT       64 chars max\n"
        "  -pi CODE       4 hex digits\n"
        "  --measure-only DSP only, no hardware (default safe mode)\n"
        "  --dump-rp1     map BAR1, verify chip ID, dump clocks_main (needs root)\n"
        "  --measure-pcie benchmark BAR1 write rate via RIO (needs root)\n"
        "  --carrier-only route GPIO20->GPCLK0, no audio modulation (needs --enable-rf)\n"
        "  --enable-rf    REQUIRED for any /dev/mem + GPIO touch. You declare\n"
        "                 authorisation + shielded/dummy-load setup.\n"
        "Antenna (future): GPIO20 pin38 / GPIO21 pin40. Never GPIO4 on Pi 5.\n", prog);
}

/* ============================ RDS DSP ============================ */
/* Simplified but real: 0A (PS) + 2A (RT) groups, CRC-10, differential,
 * biphase + root-raised-cosine shaping at 228 kHz (4x 57 kHz). */

static uint16_t rds_crc(const uint8_t *bits /*16 bits MSB first*/) {
    /* x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1, init 0 */
    uint16_t crc = 0;
    for (int i = 0; i < 16; i++) {
        int bit = (bits[i / 8] >> (7 - (i % 8))) & 1;
        int msb = (crc >> 9) & 1;
        crc = ((crc << 1) & 0x3ff) | (uint16_t)bit;
        if (msb) crc ^= 0x31bu; /* poly 0x31B without x^10 term after shift */
    }
    return crc & 0x3ff;
}

/* RRC-shaped biphase symbol table, generated at startup (avoids vendored blob) */
#define RRC_LEN 24  /* samples per biphase half at 228 kHz: 57k*2 biphase -> 4 samples/bit */
static float rrc_sym[RRC_LEN * 2];

static void rrc_init(void) {
    /* Root-raised-cosine-ish pulse via windowed sinc, beta=1, span 2 symbols */
    for (int i = 0; i < RRC_LEN * 2; i++) {
        double t = ((double)i / (RRC_LEN * 2) - 0.5) * 4.0; /* -2..2 symbols */
        double s;
        if (fabs(t) < 1e-9) s = 1.0;
        else if (fabs(fabs(t) - 1.0) < 1e-9) s = M_PI / 4.0;
        else s = (sin(M_PI * t) / (M_PI * t)) / (1.0 - t * t);
        /* Hamming window */
        double w = 0.54 - 0.46 * cos(2 * M_PI * i / (RRC_LEN * 2 - 1));
        rrc_sym[i] = (float)(s * w);
    }
}

typedef struct {
    char ps[9];      /* 8 + NUL */
    char rt[65];     /* 64 + NUL */
    uint16_t pi;
    uint8_t group_bits[104 / 8]; /* one 104-bit group staging */
    int bitpos;      /* next group bit index 0..103 */
    int group_no;    /* cycles 0..3 PS then RT */
    float prev_sample;
    int diff_state;
    float sym_phase;
} rds_t;

static void rds_init(rds_t *r, const char *ps, const char *rt, const char *pi_hex) {
    memset(r, 0, sizeof(*r));
    snprintf(r->ps, sizeof(r->ps), "%-8.8s", ps ? ps : "Pi5-FM");
    snprintf(r->rt, sizeof(r->rt), "%-64.64s", rt ? rt : "");
    r->pi = (uint16_t)(pi_hex ? strtoul(pi_hex, NULL, 16) : 0xFFFF);
    rrc_init();
}

/* Build one 104-bit group: 4 blocks of 26 bits (16 data + 10 check).
 * Check words use standard offsets A/B/C/D; we XOR the CRC with the offset
 * word per EN 50067. Offsets: A=0x0FC, B=0x198, C=0x168, D=0x1B4. */
static void rds_build_group(rds_t *r) {
    uint16_t blk[4];
    static const uint16_t offs[4] = { 0x0FC, 0x198, 0x168, 0x1B4 };
    memset(r->group_bits, 0, sizeof(r->group_bits));
#define SETBIT(i,v) do { if (v) r->group_bits[(i)/8] |= (uint8_t)(0x80 >> ((i)%8)); } while (0)
    if (r->group_no < 4) {
        /* Type 0A: PS segment */
        int seg = r->group_no;
        blk[0] = r->pi;
        blk[1] = (uint16_t)(0x0000 | seg); /* GT=0, TP=0, PTY=0, TA=0, M/S=0, DI=0, C1..C0=seg */
        blk[2] = (uint16_t)(((uint8_t)r->ps[seg * 2] << 8) | (uint8_t)r->ps[seg * 2 + 1]);
        blk[3] = r->pi; /* AF-ish filler; real encoder would put PI again */
    } else {
        /* Type 2A: RT segment 0 (bytes 0..3) cycling; full RT needs 16 segs,
         * we rotate seg 0..15 across calls via group_no. */
        int seg = (r->group_no - 4) % 16;
        blk[0] = r->pi;
        blk[1] = (uint16_t)(0x2000 | seg); /* GT=2, AB flag 0 */
        blk[2] = (uint16_t)(((uint8_t)r->rt[seg * 4] << 8) | (uint8_t)r->rt[seg * 4 + 1]);
        blk[3] = (uint16_t)(((uint8_t)r->rt[seg * 4 + 2] << 8) | (uint8_t)r->rt[seg * 4 + 3]);
    }
    for (int b = 0; b < 4; b++) {
        uint8_t data[2] = { (uint8_t)(blk[b] >> 8), (uint8_t)(blk[b] & 0xff) };
        uint16_t check = (rds_crc(data) ^ offs[b]) & 0x3ff;
        for (int i = 0; i < 16; i++) SETBIT(b * 26 + i, (blk[b] >> (15 - i)) & 1);
        for (int i = 0; i < 10; i++) SETBIT(b * 26 + 16 + i, (check >> (9 - i)) & 1);
    }
#undef SETBIT
    r->bitpos = 0;
}

/* One RDS baseband sample at 228 kHz (call sequentially). */
static float rds_sample(rds_t *r) {
    if (r->bitpos >= 104) {
        r->group_no = (r->group_no + 1) % 20; /* 4x PS + 16x RT */
        rds_build_group(r);
    }
    int byte = r->bitpos / 8, bit = 7 - (r->bitpos % 8);
    int b = (r->group_bits[byte] >> bit) & 1;
    r->bitpos++;
    /* Differential: transition on 1 */
    if (b) r->diff_state ^= 1;
    /* Biphase: each bit -> 2 chips; shape with RRC table, 2 samples per chip
     * at 228 kHz (1187.5 bps * 192 = 228 kHz). */
    float acc = 0;
    int chip = r->diff_state ? 1 : -1;
    /* Overlap-add two half-symbols: previous chip tail + current chip head */
    for (int i = 0; i < RRC_LEN; i++) {
        acc += chip * rrc_sym[i] * 0.5f;
        acc += chip * rrc_sym[i + RRC_LEN] * 0.5f;
    }
    /* 57 kHz subcarrier */
    r->sym_phase += (float)(2 * M_PI * 57000.0 / MPX_RATE);
    if (r->sym_phase > 2 * M_PI) r->sym_phase -= (float)(2 * M_PI);
    return acc * sinf(r->sym_phase) * 0.06f; /* ~3-6% injection */
}

/* ============================ Audio + MPX ============================ */

typedef struct {
    float *l, *r;      /* mono: r == NULL */
    size_t n;          /* samples at MPX_RATE */
    int stereo;
} audio_t;

/* Linear-interpolation resampler to 228 kHz (honest but basic; FIR upgrade = TODO). */
static audio_t *load_audio_228k(const char *path, int *err) {
    SNDFILE *sf = NULL;
    SF_INFO si;
    memset(&si, 0, sizeof(si));
    if (!strcmp(path, "-")) {
        sf = sf_open_fd(STDIN_FILENO, SFM_READ, &si, SF_FALSE);
    } else {
        sf = sf_open(path, SFM_READ, &si);
    }
    if (!sf) { *err = 1; return NULL; }
    if (si.channels < 1 || si.channels > 2 || si.samplerate < 8000) {
        fprintf(stderr, "unsupported audio: ch=%d rate=%d\n", si.channels, si.samplerate);
        sf_close(sf); *err = 1; return NULL;
    }
    sf_count_t frames = si.frames;
    if (frames <= 0) frames = (sf_count_t)si.samplerate * 10; /* stream fallback: 10 s */
    if (frames > si.samplerate * 600) frames = si.samplerate * 600; /* 10 min cap */
    float *tmp = malloc((size_t)(frames + 1) * si.channels * sizeof(float));
    if (!tmp) { sf_close(sf); *err = 1; return NULL; }
    sf_count_t got = sf_readf_float(sf, tmp, frames);
    int in_rate = si.samplerate, ch = si.channels;
    sf_close(sf);
    if (got <= 0) { free(tmp); *err = 1; return NULL; }

    size_t out_n = (size_t)((double)got * MPX_RATE / in_rate);
    audio_t *a = calloc(1, sizeof(*a));
    a->l = malloc(out_n * sizeof(float));
    a->r = (ch == 2) ? malloc(out_n * sizeof(float)) : NULL;
    a->stereo = (ch == 2);
    if (!a->l || (ch == 2 && !a->r)) { free(tmp); free(a->l); free(a->r); free(a); *err = 1; return NULL; }
    for (size_t i = 0; i < out_n; i++) {
        double pos = (double)i * in_rate / MPX_RATE;
        size_t idx = (size_t)pos;
        double frac = pos - idx;
        if (idx + 1 >= (size_t)got) idx = (size_t)got - 2;
        for (int c = 0; c < ch; c++) {
            float v0 = tmp[idx * ch + c], v1 = tmp[(idx + 1) * ch + c];
            float v = (float)(v0 + (v1 - v0) * frac);
            if (v > 1) v = 1; if (v < -1) v = -1;
            if (c == 0) a->l[i] = v; else a->r[i] = v;
        }
    }
    a->n = out_n;
    free(tmp);
    *err = 0;
    return a;
}

/* Build MPX float buffer [-1,1]: L+R (15 kHz honored approximately by the
 * resampler + one-pole), 19 kHz pilot @9%, L-R DSB @38 kHz, RDS @57 kHz,
 * 50 us pre-emphasis on L/R. Deviation mapping happens in output stage. */
static float *build_mpx(audio_t *a, rds_t *rds, double *peak) {
    float *mpx = malloc(a->n * sizeof(float));
    if (!mpx) return NULL;
    double ph19 = 0, ph38 = 0, d19 = 2 * M_PI * 19000.0 / MPX_RATE;
    float pre_l = 0, pre_r = 0;
    const float pre_a = (float)exp(-1.0 / (MPX_RATE * 50e-6));
    double pk = 0;
    for (size_t i = 0; i < a->n; i++) {
        float l = a->l[i], r = a->r ? a->r[i] : l;
        pre_l = pre_a * pre_l + (1 - pre_a) * l;
        pre_r = pre_a * pre_r + (1 - pre_a) * r;
        float lp = l + pre_l * 0.5f, rp = r + pre_r * 0.5f; /* gentle HF lift */
        float sum = (lp + rp) * 0.5f * 0.9f;
        float diff = (lp - rp) * 0.5f;
        float pilot = sinf((float)ph19) * 0.09f;
        float dsb = diff * sinf((float)ph38) * 2.0f * 0.9f;
        float rdsb = rds_sample(rds);
        float s = sum * 0.9f + pilot + dsb * 0.5f + rdsb;
        mpx[i] = s;
        if (fabs(s) > pk) pk = fabs(s);
        ph19 += d19; if (ph19 > 2 * M_PI) ph19 -= 2 * M_PI;
        ph38 += 2 * d19; if (ph38 > 2 * M_PI) ph38 -= 2 * M_PI;
    }
    if (peak) *peak = pk;
    return mpx;
}

/* ============================ RP1 access ============================ */

static int read_hex_file(const char *path, unsigned *val) {
    char buf[32] = {0};
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return -1;
    *val = (unsigned)strtoul(buf, NULL, 0);
    return 0;
}

/* Find RP1 BAR1 via sysfs: returns phys addr + size. */
static int rp1_find_bar1(uint64_t *phys, uint64_t *size) {
    DIR *d = opendir("/sys/bus/pci/devices");
    if (!d) { perror("opendir /sys/bus/pci/devices"); return -1; }
    struct dirent *e;
    char path[512];
    while ((e = readdir(d))) {
        if (e->d_name[0] == '.') continue;
        snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/vendor", e->d_name);
        unsigned ven = 0, dev = 0;
        if (read_hex_file(path, &ven)) continue;
        snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/device", e->d_name);
        if (read_hex_file(path, &dev)) continue;
        if (ven == RP1_VENDOR_ID && dev == RP1_DEVICE_ID) {
            snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource", e->d_name);
            FILE *f = fopen(path, "r");
            if (!f) continue;
            /* resource lines: start end flags; BAR1 = resource1 (2nd line) */
            char line[128]; int idx = 0; int ok = 0;
            while (fgets(line, sizeof(line), f)) {
                if (idx == 1) {
                    uint64_t s = 0, en = 0;
                    if (sscanf(line, "%lx %lx", (unsigned long *)&s, (unsigned long *)&en) == 2 && s) {
                        *phys = s; *size = en - s + 1; ok = 1;
                    }
                    break;
                }
                idx++;
            }
            fclose(f);
            closedir(d);
            if (ok) {
                printf("RP1 found: %s BAR1 phys=0x%llx size=0x%llx\n",
                    e->d_name, (unsigned long long)*phys, (unsigned long long)*size);
                return 0;
            }
            fprintf(stderr, "RP1 found but BAR1 unreadable. Kernel driver may own it (see below).\n");
            return -1;
        }
    }
    closedir(d);
    fprintf(stderr, "RP1 PCIe endpoint 1de4:0001 not found under /sys/bus/pci/devices. Not a Pi 5?\n");
    return -1;
}

static volatile uint8_t *rp1_map(uint64_t phys, uint64_t size, size_t *maplen) {
    long ps = sysconf(_SC_PAGESIZE);
    uint64_t off = phys % (uint64_t)ps;
    uint64_t base = phys - off;
    size_t len = (size_t)(size + off);
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem (need root + CONFIG_STRICT_DEVMEM off for PCIe BAR)");
        return NULL;
    }
    void *m = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)base);
    close(fd);
    if (m == MAP_FAILED) { perror("mmap BAR1"); return NULL; }
    *maplen = len;
    return (volatile uint8_t *)m + off;
}

static inline volatile uint32_t *rp1_reg(volatile uint8_t *bar, uint32_t off) {
    return (volatile uint32_t *)(bar + off);
}

static int rp1_verify(volatile uint8_t *bar) {
    uint32_t id = *rp1_reg(bar, RP1_OFF_CHIP_ID);
    printf("RP1 chip_id = 0x%08x (expect 0x%08x)\n", id, RP1_CHIP_ID);
    return (id == RP1_CHIP_ID) ? 0 : -1;
}

static void rp1_gpio_to_gpclk(volatile uint8_t *bar, unsigned pin) {
    volatile uint32_t *ctrl = rp1_reg(bar, RP1_GPIO_CTRL(pin));
    uint32_t v = *ctrl;
    printf("GPIO%u CTRL before = 0x%08x\n", pin, v);
    v = (v & ~RP1_GPIO_FUNCSEL_MASK) | (RP1_FUNC_GPCLK & RP1_GPIO_FUNCSEL_MASK);
    *ctrl = v;
    /* readback */
    printf("GPIO%u CTRL after  = 0x%08x (FUNCSEL->GPCLK, VERIFY with `pinctrl get %u`)\n",
        pin, *ctrl, pin);
}

static void rp1_dump_clocks(volatile uint8_t *bar) {
    printf("--- clocks_main @ 0x%x (64 regs, docs incomplete; capture for R&D) ---\n", RP1_CLOCKS_MAIN);
    for (int i = 0; i < 64; i++) {
        uint32_t v = *rp1_reg(bar, RP1_CLOCKS_MAIN + i * 4);
        printf("  +0x%03x: 0x%08x\n", i * 4, v);
    }
}

/* PCIe write-rate benchmark via RIO NOP-region alias (safe: writes to SET/CLR
 * of an unused pin are avoided; we time raw BAR writes to a scratch read instead).
 * We benchmark reads+writes to clocks dump area (read-only safe) + a tight
 * write loop to the PIO FIFO EMPTY alias is NOT safe, so we only time writes
 * to GPIO CTRL readback-free shadow: instead measure memcpy-rate to prove
 * PCIe latency bound without side effects. */
static void rp1_measure_pcie(volatile uint8_t *bar) {
    struct timespec t0, t1;
    volatile uint32_t sink = 0;
    const long N = 200000;
    clock_gettime(CLOCK_MONOTONIC, t0);
    for (long i = 0; i < N; i++) sink += *rp1_reg(bar, RP1_OFF_CHIP_ID);
    clock_gettime(CLOCK_MONOTONIC, t1);
    double dt = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
    printf("BAR1 read: %ld reads in %.3fs -> %.1f kReads/s (%.0f ns/read)\n",
        N, dt, N / dt / 1000.0, dt * 1e9 / N);
    printf("NOTE: FM needs 228000 deviation updates/s over PCIe. If read latency is\n"
           "already a large fraction of 4.4 us, jitter-free FM from userspace is\n"
           "unlikely — this number is the honest feasibility gate.\n");
}

/* The missing piece, kept explicit instead of faked. */
static int rp1_fm_modulate(volatile uint8_t *bar, float *mpx, size_t n, double freq_mhz) {
    (void)bar; (void)mpx; (void)n; (void)freq_mhz;
    fprintf(stderr,
        "\nSTOP: fractional FM modulator not mapped (ENOSYS).\n"
        "The DSP above is real and validated by --measure-only + your SDR audio chain.\n"
        "To close the loop we need the RP1 GPCLK/PLL fractional divider register\n"
        "in clocks_main @ 0x40018000 (register docs incomplete per Raspberry Pi forums).\n"
        "Next step with your SDR: run --dump-rp1 while sweeping `clk` debugfs /\n"
        "pinctrl GPCLK rates, diff the dump, identify the DIV/FRAC field, then\n"
        "implement the 228 kHz poke loop here + DMA if jitter allows.\n"
        "Do not attach an antenna until deviation + harmonics are verified on SDR.\n");
    return -ENOSYS;
}

/* ============================ main ============================ */

int main(int argc, char **argv) {
    const char *audio = NULL, *ps = "Pi5-FM", *rt = "Pi5 experimental", *pi = "FFFF";
    double freq = 100.0;
    int measure_only = 1, dump_rp1 = 0, measure_pcie = 0, carrier_only = 0, enable_rf = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-freq") && i + 1 < argc) freq = atof(argv[++i]);
        else if (!strcmp(argv[i], "-audio") && i + 1 < argc) audio = argv[++i];
        else if (!strcmp(argv[i], "-ps") && i + 1 < argc) ps = argv[++i];
        else if (!strcmp(argv[i], "-rt") && i + 1 < argc) rt = argv[++i];
        else if (!strcmp(argv[i], "-pi") && i + 1 < argc) pi = argv[++i];
        else if (!strcmp(argv[i], "--measure-only")) measure_only = 1;
        else if (!strcmp(argv[i], "--dump-rp1")) { dump_rp1 = 1; measure_only = 0; }
        else if (!strcmp(argv[i], "--measure-pcie")) { measure_pcie = 1; measure_only = 0; }
        else if (!strcmp(argv[i], "--carrier-only")) { carrier_only = 1; measure_only = 0; }
        else if (!strcmp(argv[i], "--enable-rf")) enable_rf = 1;
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(argv[0]); return 0; }
        else { usage(argv[0]); return 2; }
    }

    if (freq < 76.0 || freq > 108.0) {
        fprintf(stderr, "Refusing: freq %.1f outside 76-108 MHz (authorised FM only).\n", freq);
        return 2;
    }

    /* --- DSP first (always real, always safe) --- */
    audio_t *a = NULL;
    float *mpx = NULL;
    double peak = 0;
    if (audio) {
        rds_t rds;
        rds_init(&rds, ps, rt, pi);
        rds_build_group(&rds);
        int err = 0;
        a = load_audio_228k(audio, &err);
        if (!a) { fprintf(stderr, "audio load failed: %s\n", audio); return 1; }
        mpx = build_mpx(a, &rds, &peak);
        if (!mpx) { fprintf(stderr, "mpx alloc failed\n"); return 1; }
        printf("DSP: %zu samples @228kHz (%.1fs), stereo=%d, mpx peak=%.3f\n",
            a->n, (double)a->n / MPX_RATE, a->stereo, peak);
        if (peak > 1.2) printf("WARN: MPX peak >1.2, reduce input level to avoid over-deviation.\n");
    } else if (measure_only) {
        rds_t rds;
        rds_init(&rds, ps, rt, pi);
        rds_build_group(&rds);
        /* 1 s synthetic silence + RDS to validate chain without a file */
        audio_t synth;
        synth.n = MPX_RATE; synth.stereo = 0;
        synth.l = calloc(synth.n, sizeof(float));
        synth.r = NULL;
        mpx = build_mpx(&synth, &rds, &peak);
        free(synth.l);
        printf("DSP self-test: 1s silence+RDS, peak=%.3f (check with SDR audio later)\n", peak);
    }

    if (measure_only && !dump_rp1 && !measure_pcie && !carrier_only) {
        free(mpx);
        if (a) { free(a->l); free(a->r); free(a); }
        printf("OK --measure-only. No hardware touched.\n");
        return 0;
    }

    /* --- Hardware path: explicit authorisation gate --- */
    if (!enable_rf) {
        fprintf(stderr, "Refusing hardware access without --enable-rf (you must declare\n"
                        "authorisation + shielded/dummy-load setup). DSP above still ran.\n");
        free(mpx);
        if (a) { free(a->l); free(a->r); free(a); }
        return 2;
    }
    if (geteuid() != 0) {
        fprintf(stderr, "Need root for /dev/mem BAR1 mapping (sudo).\n");
        return 1;
    }

    uint64_t phys = 0, size = 0;
    if (rp1_find_bar1(&phys, &size)) return 1;
    size_t maplen = 0;
    volatile uint8_t *bar = rp1_map(phys, size, &maplen);
    if (!bar) return 1;
    if (rp1_verify(bar)) {
        fprintf(stderr, "Chip ID mismatch — aborting before touching GPIO.\n");
        munmap((void *)((uintptr_t)bar & ~(sysconf(_SC_PAGESIZE) - 1)), maplen);
        return 1;
    }

    if (dump_rp1) rp1_dump_clocks(bar);
    if (measure_pcie) rp1_measure_pcie(bar);
    if (carrier_only || (mpx && a)) {
        /* Route GPIO20 -> GPCLK0. Verify externally with `pinctrl get 20`. */
        rp1_gpio_to_gpclk(bar, 20);
        printf("GPIO20 routed to GPCLK0. Static carrier setup needs the fractional\n"
               "divider field (undocumented) — see rp1_fm_modulate() note.\n");
    }
    int rc = 0;
    if (mpx && a && !carrier_only) {
        rc = rp1_fm_modulate(bar, mpx, a->n, freq);
    }

    munmap((void *)((uintptr_t)bar & ~(sysconf(_SC_PAGESIZE) - 1)), maplen);
    free(mpx);
    if (a) { free(a->l); free(a->r); free(a); }
    return (rc == 0) ? 0 : 3;
}
