# Regenerates the cl.exe response files in obj/ from src/, so the build always
# sees the current translation-unit list.
$root = $PSScriptRoot
Set-Location $root

$lib = @(
  'crc', 'wav', 'pcm_wav_reader', 'mapped_file', 'bitstream_info', 'audio_frame', 'audio_block', 'emdf',
  'sync_frame', 'decoder', 'joc_decoder', 'exponents', 'bit_allocation',
  'mantissa', 'imdct', 'major_sync', 'access_unit', 'restart_header',
  'lossless_decoder', 'encoder', 'mat_dec_hmac', 'object_renderer', 'channel_layout',
  'demuxer',
  'layout', 'oamd', 'eac3_core', 'emdf_writer', 'joc_encoder', 'oamd_reframer',
  'frame_muxer', 'mkv_writer', 'mp4_writer', 'transcoder', 'truehd_file'
) | ForEach-Object { "`"$_.obj`"" }

$app = @('"main.obj"') + $lib
$tool = @('"dump_eac3.obj"') + $lib
$probe = @('"probe_truehd.obj"') + $lib
$mkv = @('"mkv_demux.obj"') + $lib
$test = @('"encoder_tests.obj"') + $lib

New-Item -ItemType Directory -Force obj | Out-Null
Set-Content obj\lib.rsp $app -Encoding ASCII
Set-Content obj\tool.rsp $tool -Encoding ASCII
Set-Content obj\probe.rsp $probe -Encoding ASCII
Set-Content obj\mkv.rsp $mkv -Encoding ASCII
Set-Content obj\test.rsp $test -Encoding ASCII
Set-Content obj\testlib.rsp $lib -Encoding ASCII
Write-Host "wrote obj\lib.rsp ($($lib.Count) objects)"
