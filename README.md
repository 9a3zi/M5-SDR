# M5-SDR

**Standalone RTL-SDR receiver and digital-mode decoder for the M5Stack Tab5 (ESP32-P4).**

M5-SDR turns the **M5Stack Tab5** into a portable software-defined radio receiver with a live FFT spectrum, scrolling waterfall, touchscreen tuning, 
demodulated audio through the built-in speaker, and several digital-mode decoders. **No PC is required during operation.** Connect a compatible 
RTL-SDR dongle to the Tab5 USB-A host port and start receiving.

> **Project status:** Active development. Receiver performance and decoder sensitivity depend on the dongle, antenna, interference, signal 
strength, and firmware version. Features described here reflect the project implementation and may change.

## Features

### Analog radio

- **WFM** — wideband FM broadcast reception.
- **NFM** — narrowband FM reception, including weather and other analog channels where reception is permitted.
- **AM, USB, LSB and CW** reception.
- Adjustable tuner gain/AGC, volume, squelch, and tuning step.
- One-touch HF, VHF and UHF band selection with remembered settings.

### Digital modes

| Mode | Function | Typical frequency / band |
| --- | --- | --- |
| **FT8** | Decodes weak-signal amateur radio messages; shows frequency, SNR, DT and text | 14.074 MHz (20 m), plus other amateur bands |
| **WSPR** | Decodes propagation beacons; shows callsign, locator and transmit power | 14.0956 MHz (20 m), plus other amateur bands |
| **RS41** | Decodes supported Vaisala radiosonde telemetry | 400–406 MHz |
| **HORUS** | Horus Binary v1, v2 and v3 telemetry decoding | Tune to the balloon's transmit frequency |
| **LoRa APRS** | Receives LoRa APRS using an external RA-02 module | According to the configured LoRa APRS channel | automatic scan sonde band | recording data to sd card |
| **APRS (AFSK1200)** | Bell 202 / AX.25 packet decoding with packet history and CRC diagnostics | 144.800 MHz / 432.500 MHz presets |

### Display and automation

- Real-time FFT spectrum and scrolling waterfall using **esp-dsp**, in landscape orientation (approximately 20 frames/s under typical conditions).
- Touchscreen tuning, step selection, band shortcuts, signal indication and squelch threshold.
- **PLAN** menu for scheduled FT8/WSPR reception (introduced in version 1.2).
- Network reporting for supported digital modes; configure station and network settings before use.

## Hardware

- **M5Stack Tab5**, featuring **ESP32-P4 + ESP32-C6** and a **720 × 1280 MIPI-DSI** display.
- Compatible **RTL-SDR USB receiver** (RTL2832U with supported tuner, including R820T/R828D variants).
- Suitable antenna for the frequency range of interest.
- Recommended: short USB-A extension cable to improve clearance and potentially reduce local interference.
- Recommended: reliable external power for the Tab5 and USB-connected hardware.
- Optional **RA-02 LoRa module** connected through the Tab5 M3 BUS; see [`LORA_NOTES.md`](LORA_NOTES.md).

> **USB host power:** USB-A VBUS is disabled by default on the board and is enabled by the firmware during startup using `bsp_tab5_set_usb_host_power()`.

## Build and flash

### Requirements

- **ESP-IDF 5.4 or newer** (developed with **ESP-IDF 5.4.4**).
- ESP-IDF toolchain and Python environment correctly installed.
- M5Stack Tab5 connected to the development computer for flashing.

From the project root, run:

```bash
idf.py set-target esp32p4   # First configuration only
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Replace `/dev/ttyACM0` with the correct port for your system. On Windows, for example:

```powershell
idf.py -p COM5 flash monitor
```

If the serial port is already configured, you can use:

```bash
idf.py flash
```

ESP-IDF Component Manager downloads managed dependencies such as **esp-dsp** and device drivers into `managed_components/`. The resolved dependency 
versions are recorded in `dependencies.lock`.

### Configure Wi-Fi and station information first, PLAN menu/settings

## Basic operation

### Tuning and bands

- **Touch tuning:** drag the spectrum or waterfall to move through the visible frequency range.
- **Fine tuning:** use `[-]` and `[+]` with the selected **STEP** value (approximately 1 kHz to 1 MHz in the general tuning interface; 
    smaller steps are available for SSB/CW).
- **Band presets:** `HF`, `VHF` and `UHF` jump to their configured starting frequencies.
- **Remembered settings:** switching away from a band and returning restores its last frequency, mode, tuning step and sample rate. 
    Pressing the currently selected band button jumps to that band's start frequency.
- **Custom presets:** edit `main/bookmarks.h` to add or adjust entries.

Default band entry points described by the current project:

| Button | Initial frequency | Mode |
| --- | ---: | --- |
| HF | 14.000 MHz | USB |
| VHF | 144.000 MHz | NFM |
| UHF | 430.000 MHz | NFM |

### Volume, squelch and gain

- Adjust **volume** and **squelch** using the on-screen controls.
- The **SIG** indicator shows received signal level; the red marker indicates the squelch threshold.
- Adjust tuner gain or AGC when strong nearby signals reduce the visibility of weak stations.

### Sample rates and HF reception

The RTL2832 does not support every arbitrary sample rate. In the implementation described here:

- **HF:** the dongle runs at **288 kS/s**, and DSP decimates to a **96 kS/s** spectrum view (approximately **96 kHz** span, **94 Hz/bin**).
- **VHF/UHF/RS41:** typically use **1.152 MS/s** (approximately **1.15 MHz** spectrum view).
- **RTL-SDR Blog V4:** HF reception below 28.8 MHz uses its built-in upconverter support.

## Analog demodulation modes

| Mode | Dial frequency interpretation | Demodulation / filtering |
| --- | --- | --- |
| WFM / NFM | Channel center | FM demodulation with mode-specific filtering |
| AM | Carrier frequency | Approximately 5 kHz low-pass; carrier-referenced AGC |
| USB / LSB | Suppressed carrier | Approximately 0.2–2.8 kHz, 255-tap FIR (Weaver) |
| CW | Carrier minus 700 Hz (700 Hz pitch) | Approximately 600 Hz Butterworth filter |
| RS41 | Center of 4800-baud GFSK signal | Approximately 8 kHz low-pass, FM discriminator, RS41 decoder |

For SSB and CW, use fine tuning steps such as **100 Hz** or **500 Hz**. The frequency display provides approximately **100 Hz** resolution. 
DSP tuning parameters, including AGC targets, filter widths and `DSP_SSB_SWAP`, are defined in `main/dsp.c`.

## Digital mode operation

### FT8

Press **FT8** to switch to the HF view and tune to **14.074 MHz** (20 m), unless already on an FT8 frequency. Repeated presses cycle 
through common FT8 frequencies:

| Band | Dial frequency |
| --- | ---: |
| 80 m | 3.573 MHz |
| 40 m | 7.074 MHz |
| 30 m | 10.136 MHz |
| 20 m | 14.074 MHz |
| 17 m | 18.100 MHz |
| 15 m | 21.074 MHz |
| 12 m | 24.915 MHz |
| 10 m | 28.074 MHz |

The receiver uses USB with an approximately **150–2950 Hz** passband; the decoder searches approximately **200–3000 Hz** of audio.

**Time synchronization:** The decoder does not depend on an accurate wall clock. While the display shows `SCANNING`, it attempts 
decoding from the latest 15 seconds of audio approximately every 2.5 seconds. A successful decode establishes the 15-second slot grid 
(`SLOT SYNC OK`), after which decoding proceeds once per slot. If decoding fails for six slots, the search resumes. Allow approximately 
**15–30 seconds** after tuning for the first full transmission and initial synchronization.

The display shows recent decoded messages with audio frequency, approximate SNR and message text. Newer messages are emphasized; 
CQ messages are highlighted.

FT8 protocol code is based on **ft8_lib** by **Kārlis Goba** (MIT); see `main/ft8lib/README_THIRD_PARTY.md`.

### WSPR

Press **WSPR** to select the WSPR receiver. The default 20 m dial frequency is **14.0956 MHz**. Repeated presses cycle through the
 configured WSPR amateur bands, including 160/80/60/40/30/20/17/15/12/10/6 m.

WSPR signals typically occupy approximately **1400–1600 Hz** of USB audio; the decoder analyzes approximately **1300–1700 Hz**.

A WSPR transmission lasts about **111 seconds**. The decoder uses a rolling spectrogram and searches for the sync pattern without 
relying on a device clock. A spot appears after enough audio has been collected and a transmission has completed. After retuning,
 allow several minutes before concluding that no usable signal is present.

The panel displays audio frequency, estimated SNR, callsign, locator and power in dBm. `OTHER n` counts less-common decoded message 
types not shown in the primary list.

**WSPR diagnostics:**

- `BUF` — collected audio history.
- `SC` — synchronization searches performed.
- `CAND` — candidate signals evaluated.
- `FANO` — decodes that converged.
- `BEST` — strongest synchronization correlation in the latest scan.
- `DROP` — audio samples lost when processing could not keep up.

**Self-test:** Tap the WSPR panel to start the built-in synthetic-signal test. The display reports its result; passing this test 
validates the internal processing path, but does not guarantee reception of weak over-the-air signals.

The WSPR decoder is described by the project as an independent implementation based on the public protocol specification rather 
than copied WSJT-X source code.

### RS41 radiosondes

Select **RS41**. If the receiver is outside the radiosonde band, it tunes to **403.000 MHz**. Locate the radiosonde signal in the 
spectrum (typically a narrow peak around 10 kHz wide). Press RS41 again to center on the strongest detected signal in view.

The telemetry panel can display:

- Radiosonde serial number and frame number.
- GPS position, altitude, speed and time when available.
- Decoder status and relevant errors.

The decoding chain includes header synchronization, de-whitening, Reed–Solomon error correction and per-block CRC checking. 
Supported position blocks include `0x7B` and newer `0x82` / `0x83` formats.

For weak signals, manually adjust tuner gain: strong signals elsewhere in the passband can cause automatic gain control to 
reduce the desired signal. Some RS41-SGM payloads are encrypted; the panel may show `POSITION ENCRYPTED`. Temperature, humidity 
and pressure are not currently decoded because they require calibration data spread across multiple frames.

### HORUS Binary v1 / v2 / v3

The **HORUS** receiver supports telemetry formats used by high-altitude balloon projects:

- **v1:** legacy 22-byte Golay-encoded packets.
- **v2:** 32-byte Golay-encoded packets.
- **v3:** supported 32/48/64/96/128-byte formats with ASN.1-encoded telemetry structures.

Tune the receiver to the active balloon transmitter frequency. Successful decoding depends on signal quality, the correct 
modulation parameters and the packet format.

The HORUS functionality uses project-specific integration code **and bundled third-party components** in `main/horuslib/` 
and `main/horusbinaryv3/`. See [Third-party software and licensing](#third-party-software-and-licensing).

### LoRa APRS

LoRa APRS reception uses an **external RA-02 LoRa transceiver** connected through the **Tab5 M3 BUS**. The module is 
separate from the RTL-SDR USB receiver.

See [`LORA_NOTES.md`](LORA_NOTES.md) for wiring, hardware requirements and radio configuration. Use the LoRa APRS mode 
to view received packets on the device.

### APRS — AX.25 / Bell 202 AFSK1200

The **APRS** mode receives conventional **1200-baud Bell 202 AFSK** packets and decodes **AX.25** frames from the 
RTL-SDR FM discriminator output.

The current user interface provides presets for **144.800 MHz** and **432.500 MHz**. Confirm the correct channel for 
your local network and region.

The decoder displays received packet text and diagnostic information:

- `OK` / `AX25 OK` — successfully decoded packets.
- `CRC ERR` — candidate frames failing CRC validation.
- `FLAGS` — recognized HDLC frame flags.
- `AFSK %` — relative tone-contrast diagnostic.
- Recent decoded packets (history).

Strong signals generally decode reliably. Weak or noisy packets can fail CRC checking; **CRC ERR is not a direct measure 
of the number of transmitted packets**. Invalid frames are not displayed as valid messages.

## Network reporting

The firmware includes network reporting capabilities for supported digital modes. Configure Wi-Fi and station identification 
in `net_cfg.h` before building. Reporting behavior and destination services depend on the enabled mode and firmware configuration; 
consult the source and settings for the exact upload paths.

## Project layout

```text
M5-SDR/
├── CMakeLists.txt
├── README.md
├── dependencies.lock
├── main/
│   ├── main.c
│   ├── sdr.c / sdr.h
│   ├── dsp.c / dsp.h
│   ├── ui.c
│   ├── audio.c
│   ├── ft8.c
│   ├── wspr.c
│   ├── rs41.c
│   ├── horus.c
│   ├── horus_decoder.c
│   ├── horus_v3.c
│   ├── lora_aprs.c
│   ├── aprs.c / aprs.h
│   ├── horuslib/
│   ├── horusbinaryv3/
│   └── ft8lib/
├── components/
│   ├── bsp/
│   └── librtlsdr/
└── LORA_NOTES.md
```

The listing is illustrative; additional project files and managed components may be present.

## Troubleshooting

| Symptom | Things to check |
| --- | --- |
| RTL-SDR not detected | USB-A host power, USB cable, dongle compatibility, external power supply |
| Spectrum visible but no audio | Mode, volume, squelch, tuned frequency and gain |
| FT8 does not decode | Correct dial frequency, full 15-second transmission, USB audio passband and signal quality |
| WSPR does not decode | Stay tuned for several minutes; inspect `BUF`, `BEST`, `CAND` and `DROP` |
| RS41 telemetry missing | Correct frequency, gain, signal strength, sonde type and encryption status |
| APRS shows CRC errors | Weak signal, interference, incorrect channel or incomplete AX.25 frames |
| Build fails | ESP-IDF version, selected `esp32p4` target, dependency resolution and configuration |

## Credits and upstream project

M5-SDR is based on the original **[hardparking/tab5-sdr](https://github.com/hardparking/tab5-sdr)** project. This repository 
adds further functionality and modifications developed by **9A3ZI and Kimi**.

Thanks to the original project authors, the open-source SDR community, the Project Horus developers and the maintainers of 
the third-party components used here.

**Important:** Code inherited from the upstream project remains subject to its applicable original license. The license of 
the upstream project should be verified before assigning a license to the combined repository.

## Third-party software and licensing

This project is licensed under the **GNU General Public License v3.0 **

M5-SDR incorporates third-party source code and libraries. 

| Component | Location / use | License information to retain or verify |
| --- | --- | --- |
| Horus modem, FSK and error correction | `main/horuslib/` | LGPL-2.1 notices and bundled license; verify file-level terms |
| FT8 library by Kārlis Goba | `main/ft8lib/ft8/` | MIT |
| KISS FFT by Mark Borgerding | `main/ft8lib/fft/`, `main/horuslib/` | BSD notices; verify exact file-level terms |
| ASN.1 runtime by Lev Walkin | `main/horusbinaryv3/` | BSD-style notices; verify exact file-level terms |
| Generated Horus Binary v3 ASN.1 structures | `main/horusbinaryv3/` | **Origin and license still require verification** |
| RTL-SDR / USB-host adaptation | `components/librtlsdr/` | **Verify the exact upstream and modified source licenses**; GPL-related obligations may apply |
| Board support | `components/bsp/` | Derived from [Tab5-HID-Device](https://github.com/hardparking/Tab5-HID-Device); verify and retain applicable notices |
| Espressif ESP-IDF / esp-dsp and drivers | Framework / managed components | Respect each component's own license, often Apache-2.0 |

### Horus Binary acknowledgements

The HORUS implementation uses project-specific code and third-party modem, error-correction and ASN.1 components. Technical 
references and upstream projects include:

- [Project Horus — horusdemodlib](https://github.com/projecthorus/horusdemodlib)
- [Project Horus — horusbinary](https://github.com/projecthorus/horusbinary)
- The Codec2/FreeDV ecosystem

Special thanks to **David Rowe**, **Mark Jessop (VK5QI)** and the Project Horus contributors.

### License status of this repository

**The license of the complete combined firmware is under review.** In particular, the original `hardparking/tab5-sdr` license, the 
exact `components/librtlsdr/` source licenses, and the Horus Binary v3 generated-code provenance must be confirmed before labeling 
the entire repository as MIT-licensed.

Any license selected for original M5-SDR contributions does **not** override third-party licenses or their redistribution obligations. 
If the repository distributes firmware binaries, additional source-code and relinking obligations may apply to some copyleft components.

## Disclaimer

This project is provided for experimentation, education and amateur radio use. Observe applicable radio regulations and local rules 
governing reception and use of received information. No warranty is provided.
