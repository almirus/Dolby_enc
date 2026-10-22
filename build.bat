@echo off
rem Direct MSVC build. CMakeLists.txt is kept for standard environments, but
rem ninja + this sandbox (named-pipe console capture) hangs, so this script
rem drives cl.exe itself. Object lists live in obj\*.rsp (see gen_rsp.ps1).
rem
rem Usage:  build.bat          full build
rem         build.bat clean    remove objects and binaries
rem         build.bat skip     relink only, reuse existing library objects
setlocal EnableDelayedExpansion
set "ROOT=%~dp0"
set "VSDEV=%ProgramFiles%\Microsoft Visual Studio\18\Insiders\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" set "VSDEV=%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" set "VSDEV=%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"
if not exist "%VSDEV%" (
  echo VsDevCmd.bat not found.
  exit /b 1
)
call "%VSDEV%" -arch=x64 -host_arch=x64
if errorlevel 1 exit /b 1
pushd "%ROOT%"
if not exist out mkdir out
if not exist obj mkdir obj
cd obj
powershell -NoProfile -ExecutionPolicy Bypass -File "%ROOT%gen_rsp.ps1"
if errorlevel 1 goto :fail

set "FLAGS=/nologo /std:c++20 /utf-8 /EHsc /O2 /W4 /permissive- /DNOMINMAX /I.. /I..\src /Fd:vc.pdb"
set "MODE=%~1"
if "%MODE%"=="" set "MODE=full"

if "%MODE%"=="clean" goto :clean
if "%MODE%"=="skip" goto :tools

:lib
echo [build] compiling library
cl %FLAGS% /c ^
 ..\src\common\crc.cpp ..\src\common\wav.cpp ..\src\common\pcm_wav_reader.cpp ..\src\common\mapped_file.cpp ^
 ..\src\eac3\bitstream_info.cpp ..\src\eac3\audio_frame.cpp ..\src\eac3\audio_block.cpp ..\src\eac3\emdf.cpp ^
 ..\src\eac3\sync_frame.cpp ..\src\eac3\decoder.cpp ..\src\eac3\joc_decoder.cpp ..\src\eac3\exponents.cpp ^
 ..\src\eac3\bit_allocation.cpp ..\src\eac3\mantissa.cpp ..\src\eac3\imdct.cpp ^
 ..\src\truehd\major_sync.cpp ..\src\truehd\access_unit.cpp ..\src\truehd\restart_header.cpp ..\src\truehd\lossless_decoder.cpp ..\src\truehd\encoder.cpp ^
 ..\src\crypto\mat_dec_hmac.cpp ..\src\render\object_renderer.cpp ..\src\dolby\channel_layout.cpp ^
 ..\src\mkv\demuxer.cpp ^
 ..\src\model\layout.cpp ..\src\model\oamd.cpp ^
 ..\src\encode\eac3_core.cpp ..\src\encode\emdf_writer.cpp ..\src\encode\joc_encoder.cpp ^
 ..\src\encode\oamd_reframer.cpp ..\src\encode\frame_muxer.cpp ..\src\encode\mkv_writer.cpp ..\src\encode\mp4_writer.cpp ..\src\encode\transcoder.cpp ..\src\encode\truehd_file.cpp
if errorlevel 1 goto :fail

:tools
echo [build] compiling executables
cl %FLAGS% ..\tests\truehd_encoder_tests.cpp encoder.obj lossless_decoder.obj access_unit.obj restart_header.obj major_sync.obj channel_layout.obj emdf.obj emdf_writer.obj mat_dec_hmac.obj /Fe:..\out\truehd-encoder-tests.exe
if errorlevel 1 goto :fail
cl %FLAGS% ..\tests\pcm_wav_reader_tests.cpp pcm_wav_reader.obj /Fe:..\out\pcm-wav-reader-tests.exe
if errorlevel 1 goto :fail
cl %FLAGS% /c ..\src\app\main.cpp
if errorlevel 1 goto :fail
cl %FLAGS% @lib.rsp /Fe:..\out\ddp-atmos.exe
if errorlevel 1 goto :fail
cl %FLAGS% ..\tests\app_version_tests.cpp /Fe:..\out\app-version-tests.exe
if errorlevel 1 goto :fail
cl %FLAGS% /c ..\src\tools\dump_eac3.cpp
if errorlevel 1 goto :fail
cl %FLAGS% @tool.rsp /Fe:..\out\dump-eac3.exe
if errorlevel 1 goto :fail
cl %FLAGS% /c ..\src\tools\probe_truehd.cpp
if errorlevel 1 goto :fail
cl %FLAGS% @probe.rsp /Fe:..\out\probe-truehd.exe
if errorlevel 1 goto :fail
cl %FLAGS% /c ..\src\tools\mkv_demux.cpp
if errorlevel 1 goto :fail
cl %FLAGS% @mkv.rsp /Fe:..\out\mkv-demux.exe
if errorlevel 1 goto :fail
cl %FLAGS% ..\tests\joc_compact_tests.cpp @testlib.rsp /Fe:..\out\joc-compact-tests.exe
if errorlevel 1 goto :fail
cl %FLAGS% /c ..\tests\encoder_tests.cpp
if errorlevel 1 goto :fail
cl %FLAGS% @test.rsp /Fe:..\out\encoder-tests.exe
if errorlevel 1 goto :fail
cd ..
popd
echo Build OK
exit /b 0

:clean
del /q *.obj >nul 2>&1
del /q ..\out\*.exe >nul 2>&1
cd ..
popd
echo Cleaned
exit /b 0

:fail
echo BUILD FAILED
cd ..
popd
exit /b 1
