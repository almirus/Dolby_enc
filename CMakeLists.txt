cmake_minimum_required(VERSION 3.20)
project(ddp_atmos_encoder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(MSVC)
  add_compile_options(/utf-8 /W4 /permissive- /EHsc)
  add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out)

add_library(ddpatmos STATIC
  src/common/crc.cpp
  src/common/wav.cpp
  src/common/pcm_wav_reader.cpp
  src/common/mapped_file.cpp
  src/dolby/channel_layout.cpp
  src/eac3/bitstream_info.cpp
  src/eac3/audio_frame.cpp
  src/eac3/audio_block.cpp
  src/eac3/emdf.cpp
  src/eac3/sync_frame.cpp
  src/eac3/decoder.cpp
  src/eac3/joc_decoder.cpp
  src/eac3/exponents.cpp
  src/eac3/bit_allocation.cpp
  src/eac3/mantissa.cpp
  src/eac3/imdct.cpp
  src/truehd/major_sync.cpp
  src/truehd/access_unit.cpp
  src/truehd/restart_header.cpp
  src/truehd/lossless_decoder.cpp
  src/truehd/encoder.cpp
  src/crypto/mat_dec_hmac.cpp
  src/render/object_renderer.cpp
  src/mkv/demuxer.cpp
  src/model/layout.cpp
  src/model/oamd.cpp
  src/encode/eac3_core.cpp
  src/encode/emdf_writer.cpp
  src/encode/oamd_reframer.cpp
  src/encode/joc_encoder.cpp
  src/encode/frame_muxer.cpp
  src/encode/mkv_writer.cpp
  src/encode/mp4_writer.cpp
  src/encode/transcoder.cpp
  src/encode/truehd_file.cpp
)
target_include_directories(ddpatmos PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(ddp-atmos src/app/main.cpp)
find_package(Threads REQUIRED)
target_link_libraries(ddp-atmos PRIVATE ddpatmos Threads::Threads)
# Refresh the translation unit containing __DATE__/__TIME__ on every build.
add_custom_target(refresh-build-clock
  COMMAND ${CMAKE_COMMAND} -E touch "${CMAKE_CURRENT_BINARY_DIR}/build_clock.hpp"
  BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/build_clock.hpp")
add_dependencies(ddp-atmos refresh-build-clock)
target_include_directories(ddp-atmos PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(ddp-atmos PRIVATE DDP_ATMOS_CMAKE_BUILD)

add_executable(dump-eac3 src/tools/dump_eac3.cpp)
target_link_libraries(dump-eac3 PRIVATE ddpatmos)

add_executable(probe-truehd src/tools/probe_truehd.cpp)
target_link_libraries(probe-truehd PRIVATE ddpatmos)

add_executable(mkv-demux src/tools/mkv_demux.cpp)
target_link_libraries(mkv-demux PRIVATE ddpatmos)

enable_testing()
add_executable(truehd-encoder-tests tests/truehd_encoder_tests.cpp)
target_link_libraries(truehd-encoder-tests PRIVATE ddpatmos)
add_test(NAME truehd-encoder-tests COMMAND truehd-encoder-tests)
add_test(NAME truehd-512-tests COMMAND truehd-encoder-tests --layout 5.1.2)
add_test(NAME truehd-514-tests COMMAND truehd-encoder-tests --layout 5.1.4)
add_executable(pcm-wav-reader-tests tests/pcm_wav_reader_tests.cpp)
target_link_libraries(pcm-wav-reader-tests PRIVATE ddpatmos)
add_test(NAME pcm-wav-reader-tests COMMAND pcm-wav-reader-tests)
add_executable(app-version-tests tests/app_version_tests.cpp)
target_include_directories(app-version-tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME app-version-tests COMMAND app-version-tests)
add_executable(encoder-tests tests/encoder_tests.cpp)
target_link_libraries(encoder-tests PRIVATE ddpatmos)
add_test(NAME encoder-tests COMMAND encoder-tests)
# The encoder regression suite also reads the local, untracked media corpus.
set_tests_properties(encoder-tests PROPERTIES
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" LABELS corpus)
add_test(NAME cli-missing-value COMMAND ddp-atmos --bitrate)
add_test(NAME cli-invalid-duration COMMAND ddp-atmos -o unused.ec3 --duration nan)
add_test(NAME cli-invalid-bitrate COMMAND ddp-atmos -o unused.ec3 --bitrate 640000junk)
set_tests_properties(cli-missing-value cli-invalid-duration cli-invalid-bitrate PROPERTIES WILL_FAIL TRUE)
add_test(NAME cli-fast-missing-layout COMMAND ddp-atmos --fast-atmos)
add_test(NAME cli-fast-invalid-layout COMMAND ddp-atmos -o unused.ec3 -i unused.thd --fast-atmos 9.1.6)
add_test(NAME cli-fast-conflicting-mode COMMAND ddp-atmos -o unused.ec3 -i unused.thd --fast-atmos 5.1.2 --no-atmos)
set_tests_properties(cli-fast-missing-layout cli-fast-invalid-layout cli-fast-conflicting-mode PROPERTIES WILL_FAIL TRUE)

add_executable(joc-compact-tests tests/joc_compact_tests.cpp)
target_link_libraries(joc-compact-tests PRIVATE ddpatmos)
add_test(NAME joc-lossless-compaction COMMAND joc-compact-tests)
