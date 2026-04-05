cmake_minimum_required(VERSION 3.2)

project(dablast LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Build options ─────────────────────────────────────────────────────────────
option(RTLSDR    "Compile with RTL-SDR support"    ON)
option(AIRSPY    "Compile with Airspy support"     OFF)
option(SOAPYSDR  "Compile with SoapySDR support"   OFF)
option(KISS_FFT  "Use KISS FFT instead of FFTW3f"  OFF)

add_definitions(-Wall -O3 -DDABLIN_AAC_FAAD2 -ffile-prefix-map=${CMAKE_SOURCE_DIR}=.)

# ── CMake modules ─────────────────────────────────────────────────────────────
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

# ── Find dependencies ─────────────────────────────────────────────────────────
if(KISS_FFT)
    add_definitions(-DKISSFFT)
    set(fft_sources "${CMAKE_CURRENT_SOURCE_DIR}/lib/kiss_fft/kiss_fft.c")
    set(KISS_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/lib/kiss_fft")
else()
    find_package(FFTW3f REQUIRED)
    set(fft_sources "")
    set(KISS_INCLUDE_DIRS "")
endif()

find_package(Faad)
find_package(MPG123 REQUIRED)
find_package(Threads REQUIRED)

if(NOT FAAD_FOUND)
    message(FATAL_ERROR
        "libfaad not found.\n"
        "Install with:  sudo apt install libfaad-dev")
endif()

if(RTLSDR)
    find_package(LibRTLSDR)
    if(LIBRTLSDR_FOUND)
        add_definitions(-DHAVE_RTLSDR)
    else()
        message(WARNING
            "librtlsdr not found – RTL-SDR support disabled.\n"
            "Install with:  sudo apt install librtlsdr-dev")
        set(RTLSDR OFF)
    endif()
endif()

if(AIRSPY)
    find_package(LibAIRSPY REQUIRED)
    add_definitions(-DHAVE_AIRSPY)
endif()

if(SOAPYSDR)
    find_package(SoapySDR NO_MODULE REQUIRED)
    set(CMAKE_CXX_STANDARD 14)
    add_definitions(-DHAVE_SOAPYSDR)
endif()

# ── Include paths ─────────────────────────────────────────────────────────────
set(dablast_includes
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/various
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/input
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/fec
    ${FFTW3F_INCLUDE_DIRS}
    ${KISS_INCLUDE_DIRS}
)
if(FAAD_INCLUDE_DIRS)
    list(APPEND dablast_includes ${FAAD_INCLUDE_DIRS})
endif()
if(LIBRTLSDR_FOUND)
    list(APPEND dablast_includes ${LIBRTLSDR_INCLUDE_DIRS})
endif()
if(SoapySDR_FOUND)
    list(APPEND dablast_includes ${SoapySDR_INCLUDE_DIRS})
endif()
include_directories(${dablast_includes})

# ── welle.io backend sources ──────────────────────────────────────────────────
set(backend_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/dab-audio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/decoder_adapter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/dab_decoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/dabplus_decoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/charsets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/dab-constants.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/mot_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/pad_decoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/eep-protection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/fib-processor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/fic-handler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/msc-handler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/freq-interleaver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/ofdm-decoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/ofdm-processor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/phasereference.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/phasetable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/tii-decoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/protTables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/radio-receiver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/tools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/uep-protection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/backend/viterbi.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/various/Socket.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/various/Xtan2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/various/channels.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/various/fft.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/various/profiling.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/fec/decode_rs_char.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/fec/encode_rs_char.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/fec/init_rs_char.c
)

set(input_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/input/input_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/input/null_device.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/input/raw_file.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/input/rtl_tcp.cpp
)

if(LIBRTLSDR_FOUND)
    list(APPEND input_sources ${CMAKE_CURRENT_SOURCE_DIR}/lib/input/rtl_sdr.cpp)
endif()

if(LIBAIRSPY_FOUND)
    list(APPEND input_sources ${CMAKE_CURRENT_SOURCE_DIR}/lib/input/airspy_sdr.cpp)
endif()

if(SoapySDR_FOUND)
    list(APPEND input_sources ${CMAKE_CURRENT_SOURCE_DIR}/lib/input/soapy_sdr.cpp)
endif()

# ── dablast executable ────────────────────────────────────────────────────────
add_executable(dablast
    src/dablast.cpp
    src/sink_registry.cpp
    ${backend_sources}
    ${input_sources}
    ${fft_sources}
)

set(dablast_libs
    ${FFTW3F_LIBRARIES}
    ${FAAD_LIBRARIES}
    ${MPG123_LIBRARIES}
    Threads::Threads
)
if(LIBRTLSDR_FOUND)
    list(APPEND dablast_libs ${LIBRTLSDR_LIBRARIES})
endif()
if(LIBAIRSPY_FOUND)
    list(APPEND dablast_libs ${LIBAIRSPY_LIBRARIES})
endif()
if(SoapySDR_FOUND)
    list(APPEND dablast_libs ${SoapySDR_LIBRARIES})
endif()

target_link_libraries(dablast ${dablast_libs})

install(TARGETS dablast RUNTIME DESTINATION bin)
