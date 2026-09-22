# PiWave is available at https://piwave.xyz
# Licensed under GPLv3.0, main GitHub repository at https://github.com/douxxtech/piwave/
# piwave/backends/pi5_fm.py : EXPERIMENTAL Raspberry Pi 5 native GPIO RF backend
#
# Status: SCAFFOLD ONLY - no working transmitter yet.
# The old backends (pi_fm_rds, fm_transmitter) bit-bang the BCM SoC GPCLK/DMA
# to GPIO4. On Pi 5 all header GPIO moved to the RP1 southbridge over PCIe,
# so that path is physically gone. A real Pi 5 backend must drive RP1
# GPCLK0/GPCLK1 (GPIO20/GPIO21, NOT GPIO4) or RP1 PIO via PCIe BAR1 mapping.
# See c_src/pi5_fm/README.md for the R&D plan and references.
#
# This class exists so PiWave can detect Pi 5, fail fast on BCM backends,
# and auto-select this backend once its C binary (pi5_fm_rds) is built.

from .base import Backend


class Pi5FmBackend(Backend):
    # This is the only backend allowed on Pi 5.
    supports_pi5 = True

    # RP1 GPCLK outputs live on GPIO20/21, NOT GPIO4 (pin 7).
    antenna_pins = "GPIO20 (GPCLK0, pin 38) or GPIO21 (GPCLK1, pin 40)"

    @property
    def name(self):
        return "pi5_fm"

    @property
    def frequency_range(self):
        return (76.0, 108.0)

    @property
    def supports_rds(self):
        return True

    @property
    def supports_live_streaming(self):
        return False

    @property
    def supports_loop(self):
        return False

    def _get_executable_name(self):
        return "pi5_fm_rds"

    def _get_search_paths(self):
        return ["/opt/PiWave/pi5_fm", "/opt", "/usr/local/bin", "/usr/bin", "/bin", "/home"]

    def build_command(self, wav_file: str, loop: bool) -> list:
        cmd = [
            self.required_executable,
            '-freq', str(self.frequency),
            '-audio', wav_file
        ]

        if self.ps:
            cmd.extend(['-ps', self.ps])
        if self.rt:
            cmd.extend(['-rt', self.rt])
        if self.pi:
            cmd.extend(['-pi', self.pi])

        return cmd

    def build_live_command(self):
        return None  # not supported yet
