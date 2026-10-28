# Horus v1 437.600 MHz test preset

This build changes the HORUS button preset to **437.600 MHz** (100 baud, 4FSK) for the user's live test transmitter. On first entering HORUS mode the receiver retunes to 437.600 MHz. Pressing HORUS while already in that mode preserves manual frequency corrections. The RTL-SDR USB-A path is unchanged, and the separate RA-02 LoRa APRS receiver is unchanged.

Official implementation reference: https://github.com/projecthorus/horusdemodlib (C modem/Golay components, LGPL-2.1). Integrating those parts requires a separate ESP-IDF port, validation against known samples, and license compliance. No third-party Horus library source was copied into this archive.

ESP-IDF compilation and hardware reception have not been tested in this environment.
