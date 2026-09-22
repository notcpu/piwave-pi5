# pi5_fm — Pi 5 native FM research tool (experimental)

Not a working transmitter yet. Old backends drive BCM GPCLK to GPIO4, which
has no route on Pi 5 (GPIO moved to RP1 over PCIe). This tool provides the
real groundwork: FM MPX + RDS DSP, RP1 BAR1 mapping, GPIO20/21 (GPCLK0/1)
setup, clock dumps, and a PCIe benchmark. Final RF modulation is still
`ENOSYS` pending the undocumented RP1 divider register.

Build on Pi 5: `sudo apt install -y build-essential libsndfile1-dev && make`

Try: `./pi5_fm_rds --measure-only -audio song.wav` (safe, no hardware).

Needs `--enable-rf` + root for hardware. Authorised use only, dummy load first.
Antenna (future): GPIO20 pin 38 / GPIO21 pin 40, never GPIO4.
