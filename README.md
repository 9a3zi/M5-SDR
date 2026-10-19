# Tab5-SDR

A standalone RTL-SDR receiver that runs entirely on an M5Stack Tab5
(ESP32-P4) — no PC required. Plug an RTL-SDR dongle into the Tab5's USB-A host
port and get a live spectrum + waterfall, touch tuning, and demodulated audio
out the built-in speaker

- **Wideband FM** (broadcast) and **narrowband FM**.
- **AM**, **USB**, **LSB** and **CW** (see "Modes" below).
- **FT8** decoder (weak-signal amateur digital mode, 14.074 MHz etc.): list of decoded messages with frequency, SNR and DT.
- **RS41 radiosonde** decoder (Vaisala weather balloons, 400–406 MHz): serial, frame no., GPS position/altitude/speed/time.
- On-device **FFT spectrum + scrolling waterfall** (esp-dsp), landscape, ~20 fps.
- **Touch UI**: drag the waterfall to scan, fine-step tuning, volume, squelch,
  and one-tap **band buttons** (HF 14 MHz USB, VHF 144 MHz NFM, UHF 430 MHz NFM).

## Hardware

- **M5Stack Tab5** (ESP32-P4 + ESP32-C6), 720×1280 MIPI-DSI panel.
- An **RTL-SDR** dongle V4 recommended,(RTL2832U + R820T/R828D) on the **USB-A**   port, external power suppl for tab5 is recommended but working fine on usb   power.

## Build & flash

Requires **ESP-IDF v5.4+** (developed on v5.4.4).

```bash
idf.py set-target esp32p4    # first time only
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

The IDF component manager fetches `esp-dsp`, the LCD/touch/codec drivers, etc.
into `managed_components/` (pinned by `dependencies.lock`).

## Using it

- **Tune:** drag the spectrum/waterfall to scan (~100 MHz per swipe); `[-]`/`[+]`
  step by the selected **STEP** (1 k … 1 M, tap STEP to cycle); or tap a band button.
- **Mode:** bottom-left row `WFM NFM AM USB LSB CW`. Broadcast FM = WFM; NOAA and
  public-safety voice = NFM (the band buttons set this automatically).
- **Volume / Squelch:** `[-]`/`[+]`. The SIG meter (top-right) shows signal level;
  the red marker is the squelch threshold — audio mutes below it.
- **Bands:** `HF` (14.000 MHz, USB, 500 Hz step) / `VHF` (144.000 MHz, NFM) / `UHF` (430.000 MHz, NFM) jump to the band start. Add entries to the lists in `main/bookmarks.h` to make a button cycle through more presets.
  Each band **remembers its last frequency, mode, step and sample rate**: coming back from another band restores them (e.g. HF back at 7.520 MHz LSB). Pressing the button of the band you are already in jumps to its start frequency.
- **HF sample rate:** the RTL2832 cannot run at ~100 kS/s (it only accepts 225001–300000 and 900001–3200000 Hz). HF therefore runs the dongle at 288 kS/s and the DSP decimates to 96 kS/s: the spectrum spans 96 kHz at 94 Hz per bin. VHF/UHF/RS41 use 1.152 MS/s (1.15 MHz view).



## Modes

| Mode | Dial frequency means | Filter |
|------|----------------------|--------|
| WFM / NFM | channel centre | unchanged |
| AM | carrier | 5 kHz low-pass, carrier-referenced AGC |
| USB / LSB | suppressed carrier (like a normal SSB radio) | ~0.2–2.8 kHz, 255-tap FIR (Weaver) |
| CW | carrier **minus 700 Hz** (tone pitch 700 Hz) | ~600 Hz wide Butterworth |
| RS41 | centre of the 4800 baud GFSK signal | 8 kHz low-pass → FM discriminator → `rs41.c` |

For SSB/CW use the 100 / 500 Hz steps (STEP button); the frequency readout shows
100 Hz resolution. On an RTL-SDR Blog V4 the HF bands (< 28.8 MHz) work through
the built-in upconverter. Tunables (AGC target, filter widths, `DSP_SSB_SWAP`)
are `#define`s at the top of `main/dsp.c`.

### FT8

Press **FT8** (right end of the mode row). It switches to the 96 kHz HF view and
tunes to 14.074 MHz (20 m) unless you are already on an FT8 frequency; press it
again to step through 80/40/30/20/17/15/12/10 m (3.573, 7.074, 10.136, 14.074,
18.100, 21.074, 24.915, 28.074 MHz). The mode is USB with a ~150-2950 Hz passband;
the dial can be a little off — the decoder searches 200-3000 Hz of audio.

The device has **no clock**, so the decoder finds the 15-second slot timing
itself: while the panel says `SCANNING` it tries a decode on the latest 15 s of
audio every ~2.5 s; the first successful decode fixes the slot grid, then it
decodes once per slot (`SLOT SYNC OK`, `T` = seconds into the slot). After the
first decode it keeps the timing by counting audio samples. If nothing decodes for
6 slots it scans again. Expect a decode only after the first full transmission
heard, i.e. up to ~15-30 s after tuning in.

Columns: audio frequency (Hz), approximate SNR (dB in 2500 Hz), DT (seconds vs.
the slot grid *as set by the first decode*, so it is relative to the stations you
hear, not to UTC), message. Newest slot is bright, CQ calls yellow.

Protocol code (LDPC, CRC, message packing) is `ft8_lib` by Karlis Goba (MIT),
in `main/ft8lib/`; see `main/ft8lib/README_THIRD_PARTY.md`.

### RS41 radiosondes

Select `RS41` (jumps to 403.000 MHz if you are not in 400–406 MHz). Find the
sonde on the spectrum (a ~10 kHz wide hump; radiosondy.info / sondehub.org list
active frequencies), press **SNAP** to centre on the peak, and the waterfall
area turns into a telemetry panel. Decoding: header sync (both polarities) →
de-whitening → Reed-Solomon RS(255,231) ×2 → CRC16 per block. The bit clock is tracked during the frame (±0.8 % baud error). Blocks are parsed in
sequence by type (position `0x7B`, or `0x82`/`0x83` on newer sondes). If the
sonde is weak, use the **gain button** (AGC / 20 … 49.6 dB): the tuner AGC can
turn the gain down because of strong signals elsewhere, leaving the sonde only a
few ADC counts high. RS41-SGM sondes encrypt their payload (block 0x80A7): the
panel then shows serial/frame number but `POSITION ENCRYPTED`. Temperature /
humidity / pressure are not decoded (they need the per-sonde calibration that
is spread over many frames).


| Module | Role |
|--------|------|
| `main/sdr.c`   | librtlsdr device + async IQ stream → PSRAM ring buffer |
| `main/dsp.c`   | WFM/NFM demod → audio; windowed FFT → spectrum; squelch |
| `main/audio.c` | ES8388 speaker output (esp_codec_dev) via a StreamBuffer |
| `main/ui.c`    | landscape touch UI; PPA-rotated rendering |
| `main/bookmarks.h` | HF / VHF / UHF band presets (mode, sample rate, step) |
| `components/bsp/` | M5Stack Tab5 board support (panel, touch, I/O, audio, USB power) |
| `components/librtlsdr/` | librtlsdr ported onto the ESP-IDF USB host stack |

Rendering runs at half resolution (640×360) and uses the P4 **PPA** to rotate/scale
onto the portrait panel — full-res rotation costs ~46 ms/frame, half-res is ~4×
cheaper.

## Credits & license

This project is licensed under the **GNU General Public License v2.0 or later**
(see [LICENSE](LICENSE)), as required by librtlsdr.

- **[librtlsdr](https://github.com/steve-m/librtlsdr)** — Steve Markgraf,
  Dimitri Stolnikov, et al. (GPL-2.0+). The ESP-IDF USB-host port in
  `components/librtlsdr/` derives from
  [XTR1984/xtrsdr](https://github.com/XTR1984/xtrsdr), which runs librtlsdr on
  the ESP32 USB host stack.
- **Board support** (`components/bsp/`) — derived from
  [Tab5-HID-Device](https://github.com/hardparking/Tab5-HID-Device) by
  Hiroki Kawakami (MIT).
- **esp-dsp**, **esp_codec_dev**, and the LCD/touch drivers — Espressif (Apache-2.0).

Modifications to the vendored librtlsdr sources are documented inline (e.g. the
`usb_host` shim and a missing `<string.h>` include for GCC 14).
