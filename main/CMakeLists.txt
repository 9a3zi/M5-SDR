idf_component_register(
    SRCS
        "main.c" "sdr.c" "dsp.c" "aprs.c" "audio.c" "ui.c" "rs41.c" "rs41_log.c" "ft8.c" "wspr.c"
        "wifi_net.c" "station_settings.c" "psk.c" "bandplan.c" "lora_aprs.c" "horus.c" "horus_decoder.c"
        "horus_v3.c"
        "horuslib/fsk.c" "horuslib/kiss_fft.c" "horuslib/kiss_fftr.c"
        "horuslib/mpdecode_core.c" "horuslib/H_256_768_22.c" "horuslib/H_128_384_23.c"
        "horuslib/golay23.c" "horuslib/phi0.c" "horuslib/horus_api.c" "horuslib/horus_l2.c"
        "ft8lib/ft8/constants.c" "ft8lib/ft8/crc.c" "ft8lib/ft8/decode.c"
        "ft8lib/ft8/encode.c" "ft8lib/ft8/ldpc.c" "ft8lib/ft8/message.c"
        "ft8lib/ft8/text.c" "ft8lib/fft/kiss_fft.c" "ft8lib/fft/kiss_fftr.c"
        "horusbinaryv3/AdditionalSensorType.c"
        "horusbinaryv3/AdditionalSensors.c"
        "horusbinaryv3/BIT_STRING.c"
        "horusbinaryv3/BOOLEAN.c"
        "horusbinaryv3/BitFlags.c"
        "horusbinaryv3/CustomFieldValues.c"
        "horusbinaryv3/GnssPowerSaveState.c"
        "horusbinaryv3/IA5String.c"
        "horusbinaryv3/INTEGER.c"
        "horusbinaryv3/MilliVoltSensors.c"
        "horusbinaryv3/NativeEnumerated.c"
        "horusbinaryv3/NativeInteger.c"
        "horusbinaryv3/NativeReal.c"
        "horusbinaryv3/OCTET_STRING.c"
        "horusbinaryv3/OPEN_TYPE.c"
        "horusbinaryv3/REAL.c"
        "horusbinaryv3/Telemetry.c"
        "horusbinaryv3/TemperatureSensors.c"
        "horusbinaryv3/asn_SEQUENCE_OF.c"
        "horusbinaryv3/asn_SET_OF.c"
        "horusbinaryv3/asn_application.c"
        "horusbinaryv3/asn_bit_data.c"
        "horusbinaryv3/asn_codecs_prim.c"
        "horusbinaryv3/asn_internal.c"
        "horusbinaryv3/asn_random_fill.c"
        "horusbinaryv3/ber_decoder.c"
        "horusbinaryv3/ber_tlv_length.c"
        "horusbinaryv3/ber_tlv_tag.c"
        "horusbinaryv3/constr_CHOICE.c"
        "horusbinaryv3/constr_SEQUENCE.c"
        "horusbinaryv3/constr_SEQUENCE_OF.c"
        "horusbinaryv3/constr_SET_OF.c"
        "horusbinaryv3/constr_TYPE.c"
        "horusbinaryv3/constraints.c"
        "horusbinaryv3/der_encoder.c"
        "horusbinaryv3/per_decoder.c"
        "horusbinaryv3/per_encoder.c"
        "horusbinaryv3/per_opentype.c"
        "horusbinaryv3/per_support.c"
        "horusbinaryv3/xer_decoder.c"
        "horusbinaryv3/xer_encoder.c"
        "horusbinaryv3/xer_support.c"
    INCLUDE_DIRS "." "ft8lib" "horusbinaryv3"
    REQUIRES bsp librtlsdr usb esp_timer esp_driver_ppa espressif__esp-dsp
             esp_driver_spi esp_driver_gpio esp_wifi espressif__esp_wifi_remote
             esp_netif esp_event nvs_flash lwip fatfs sdmmc
)

# The ASN.1 headers are also included by horus_v3.c, so the macro must be
# visible throughout the main component, not only to the ASN.1 .c files.
target_compile_definitions(${COMPONENT_LIB} PRIVATE ASN_DISABLE_OER_SUPPORT=1)

# Third-party FT8 / KISS FFT code.
set_source_files_properties(
    "ft8lib/ft8/constants.c" "ft8lib/ft8/crc.c" "ft8lib/ft8/decode.c"
    "ft8lib/ft8/encode.c" "ft8lib/ft8/ldpc.c" "ft8lib/ft8/message.c"
    "ft8lib/ft8/text.c" "ft8lib/fft/kiss_fft.c" "ft8lib/fft/kiss_fftr.c"
    PROPERTIES COMPILE_FLAGS "-O2 -Wno-error -Wno-format -Wno-unused-variable -Wno-unused-function"
)

# Digital decoders.
set_source_files_properties("ft8.c" "wspr.c" PROPERTIES COMPILE_FLAGS "-O2")

# HORUS V1/V2 library.
set_source_files_properties(
    "horuslib/fsk.c" "horuslib/horus_api.c"
    "horuslib/horus_l2.c" "horuslib/golay23.c"
    PROPERTIES COMPILE_FLAGS "-O2 -Wno-error"
)

# HORUS V3 ASN.1 biblioteka
file(GLOB HORUS_V3_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/horusbinaryv3/*.c"
)

set_source_files_properties(
    ${HORUS_V3_SOURCES}
    PROPERTIES COMPILE_FLAGS
        "-O2 -Wno-error=array-bounds -Wno-error=misleading-indentation -DASN_DISABLE_OER_SUPPORT=1"
)
# HORUS L2 RX decoder: interleaving and scrambling.
set_source_files_properties(
    "horuslib/horus_l2.c"
    PROPERTIES COMPILE_FLAGS "-O2 -Wno-error -DHORUS_L2_RX -DINTERLEAVER -DSCRAMBLER"
)
