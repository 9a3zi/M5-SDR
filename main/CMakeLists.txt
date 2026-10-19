idf_component_register(
    SRCS "main.c" "sdr.c" "dsp.c" "audio.c" "ui.c" "rs41.c" "ft8.c"
         "ft8lib/ft8/constants.c" "ft8lib/ft8/crc.c" "ft8lib/ft8/decode.c" "ft8lib/ft8/encode.c"
         "ft8lib/ft8/ldpc.c" "ft8lib/ft8/message.c" "ft8lib/ft8/text.c"
         "ft8lib/fft/kiss_fft.c" "ft8lib/fft/kiss_fftr.c"
    INCLUDE_DIRS "." "ft8lib"
    REQUIRES bsp librtlsdr usb esp_timer esp_driver_ppa espressif__esp-dsp)

# Third-party FT8 / KISS FFT code (MIT / BSD): don't let its warnings fail the build.
set_source_files_properties(
    "ft8lib/ft8/constants.c" "ft8lib/ft8/crc.c" "ft8lib/ft8/decode.c" "ft8lib/ft8/encode.c"
    "ft8lib/ft8/ldpc.c" "ft8lib/ft8/message.c" "ft8lib/ft8/text.c"
    "ft8lib/fft/kiss_fft.c" "ft8lib/fft/kiss_fftr.c"
    PROPERTIES COMPILE_FLAGS "-Wno-error -Wno-format -Wno-unused-variable -Wno-unused-function")
