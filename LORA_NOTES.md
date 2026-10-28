# Tab5 two independent receivers

- SX1278 RA-02 SPI on external M3 bus: SCK=5 MOSI=18 MISO=19 CS=45 RST=4 DIO0=2. LoRa APRS can stay enabled while the RTL-SDR operates in any mode, including HORUS. It is independent of the RTL-SDR demodulator.
- LoRa APRS defaults: 433.775 MHz, SF12, BW125, coding rate 4/5, explicit header, CRC, sync word 0x12. These must match the transmitter. Reception only; no APRS-IS upload.
- HORUS default tune is 437.600 MHz when outside 430–440 MHz, **not a guaranteed local frequency**. Adjust to the actual sonde frequency.
- Check GPIO assignments and any Tab5 board reservations before attaching hardware.
