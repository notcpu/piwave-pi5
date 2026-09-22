# pi5_fm — EXPERIMENTAL native Raspberry Pi 5 GPIO RF (R&D tool)

Status: **RESEARCH TOOL, NOT A WORKING TRANSMITTER YET**. `pi5_fm.c` is real
code (FM MPX + RDS DSP, RP1 BAR1 mapping, clock dumps, PCIe benchmark) but the
final fractional-modulation poke is still `ENOSYS` — the RP1 divider register
is undocumented. See `rp1_fm_modulate()` in the source.

## Why the old code can't be patched

- `PiFmRds` (`pi_fm_rds.c`) and `fm_transmitter` (`transmitter.cpp`) `mmap`
  BCM283x/2711 peripherals (`0x20000000` / mailbox) and abuse the SoC general
  clock (GPCLK0) + PWM/DMA to put ~76–108 MHz FM on **GPIO4 (pin 7)**.
- On Pi 5 header GPIO lives on the **RP1 southbridge over PCIe** (see
  `RP-008370-DS-1-rp1-peripherals.pdf`). BCM GPCLK has no route to the pins.
  `fm_transmitter` README says it plainly: up to Pi 4, Pi 5 uses a different
  peripheral chip.

## What RP1 does have

- `GPCLK[0]` on **GPIO20 (pin 38)**, `GPCLK[1]` on **GPIO21 (pin 40)** — NOT GPIO4.
  See RP1 datasheet Table 4, GPIO function select.
- PWM block (2 instances, 4 channels on bank 0) + PIO block (~150 MHz).
- Clocks from `pll_audio_pri_ph / pll_video_sec / xosc` via `clk-rp1` driver,
  PWM rate in tree is e.g. `6144000` — far from the fractional 228 kHz FM modulator.
- Direct BAR1 access pattern is proven by `praktronics/rpi5-rp1-gpio`
  (map RP1 PCIe BAR1 via `/dev/mem`, poke `GPIOx_CTRL`), and Pi 5 PWM on forums
  needs `dtoverlay=pwm*` + `pinctrl`. FM needs much tighter timing than LEDs/PWM.

## R&D plan (hardware required)

1. Map RP1 BAR1 from Linux, configure GPIO20 → `GPCLK0` (FUNCSEL `a3` per Table 4).
2. Find an RP1 clock path that can hold ~100 MHz and be FM-modulated at 228 kHz
   (fractional divider update via DMA or tight loop over PCIe — jitter is the killer).
3. Re-implement FM MPX + RDS (`rds.c`, `fm_mpx.c` logic) on top of the new clock.
4. Validate with RTL-SDR + scope: carrier accuracy, deviation, harmonics, RDS groups.
   Expect mailbox IPC (~10 µs) to be too slow — likely needs kernel driver, like
   `rpi_ws281x` needed a `rp1_ws281x_pwm.ko` + overlay for Pi 5.

References:
- RP1 peripherals: `RP-008370-DS-1-rp1-peripherals.pdf`, ch. 3.1 (GPIO FUNCSEL), 3.4 (PWM)
- `praktronics/rpi5-rp1-gpio` — BAR1 mapping example
- `jgarff/rpi_ws281x` wiki “Raspberry Pi 5 Support” — why a kernel module was needed
- Raspberry Pi forums: “Pi5 - PWM on GPIO 18”, `clk-rp1.c` parents/dividers

## Safety

Do not attach an antenna until you understand harmonics/bandwidth — square-wave
VHF is rich in harmonics and unlicensed TX is illegal in most countries.
Use a shielded dummy load / direct coax to receiver for tests.

## Build & run (on Pi 5 only)

```
sudo apt install -y build-essential libsndfile1-dev
make
./pi5_fm_rds --measure-only -audio song.wav
sudo ./pi5_fm_rds --dump-rp1 --enable-rf
sudo ./pi5_fm_rds --measure-pcie --enable-rf
sudo ./pi5_fm_rds --enable-rf --carrier-only -freq 100.0
```

Send back: `--dump-rp1` output, `--measure-pcie` numbers, SDR screenshots.
That data decides whether userspace FM is feasible or a kernel driver is mandatory.

## Python side

`piwave/backends/pi5_fm.py:Pi5FmBackend` expects a `pi5_fm_rds` binary with
`pi_fm_rds`-compatible flags (`-freq -audio -ps -rt -pi`). Once a real binary
exists, install it to `/opt/PiWave/pi5_fm/pi5_fm_rds` and run
`python3 -m piwave search`.
