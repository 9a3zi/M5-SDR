# ft8_lib (third-party)

`ft8/` and `fft/` are taken unchanged from **ft8_lib** by Kārlis Goba
(https://github.com/kgoba/ft8_lib), MIT licensed — see `LICENSE` in this folder.
`fft/` is KISS FFT (Mark Borgerding, BSD-3-Clause), as bundled by ft8_lib.

They provide the FT8 protocol tables, the LDPC(174,91) decoder, CRC-14, message
packing/unpacking and the Costas-sync candidate search. The glue that feeds them
(audio capture, waterfall, slot synchronisation, message list) is in `../ft8.c`.
