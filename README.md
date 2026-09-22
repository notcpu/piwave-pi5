<h1>Pi 5 PiWave</h1>
</div>

**PiWave** is a Python module for FM broadcasting on Raspberry Pi with RDS support. It picks the best transmission backend for your frequency automatically.

Backends: `pi_fm_rds` (76–108 MHz, RDS, Pi 1–4/Zero), `fm_transmitter` (1–250 MHz, live, Pi 1–4), `pi5_fm` (76–108 MHz, RDS, Pi 5 experimental — see `c_src/pi5_fm/README.md`).

Antenna: GPIO4 pin 7 on Pi 1–4, GPIO20 pin 38 / GPIO21 pin 40 on Pi 5.

```python
from piwave import PiWave
pw = PiWave(frequency=100.0, ps="MyRadio", rt="Hello")
pw.play("song.mp3")
```

> [!WARNING]
> Transmitting RF may require authorisation and is illegal without it in most countries. Use a dummy load / shielded setup, verify with SDR. Authors accept no liability.
> **Credits to DouxxTech for the original repository**
