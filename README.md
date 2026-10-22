# Dolby_enc — Dolby Atmos Encoder

> **An experimental C++ encoder for Dolby Digital Plus / E-AC-3 Atmos and
> TrueHD channel-based immersive audio.**

[![License](https://img.shields.io/badge/License-PolyForm%20Noncommercial%201.0.0-blue.svg)](LICENSE)
[![Build](https://github.com/almirus/Dolby_enc/actions/workflows/build.yml/badge.svg)](https://github.com/almirus/Dolby_enc/actions/workflows/build.yml)

**Dolby_enc** provides the `ddp-atmos` command-line utility for multichannel
encoding, TrueHD Atmos transcoding, and source bed/object export. It includes
native C++ bitstream parsing, lossless decoding, spatial rendering,
E-AC-3/JOC encoding, and experimental TrueHD CBI encoding.

The project is intended for developers, audio engineers, researchers, and
enthusiasts working with immersive audio and codec interoperability.

> **Note:** This is an independent project, not affiliated with or endorsed
> by Dolby Laboratories or LG Electronics. Third-party names and trademarks
> describe technical compatibility or the subject of research.

---

## Overview

- Encode multichannel WAV or decoded TrueHD audio to E-AC-3.
- Reconstruct Atmos OAMD/JOC with the experimental `--atmos` path.
- Render TrueHD Atmos to fixed 5.1.2, 5.1.4, or 7.1.4 layouts with `--fast-atmos`.
- Encode PCM WAV to experimental lossless TrueHD CBI with static height channels.
- Export source bed/object audio and metadata from supported TrueHD/E-AC-3 Atmos.
- Write raw E-AC-3, Matroska, MP4, or IEC 61937 output; TrueHD supports raw
  `.thd` and Matroska output.

## Project Scope and Status

This is an evolving research utility. JOC reconstruction and compatibility
with hardware players remain under validation. `--fast-atmos` replaces the
source object scene with a fixed channel bed. TrueHD CBI encodes existing
height channels; it does not generate moving objects or upmix surround audio
into new height content.

The current version is **0.0.1 alfa**.

## Requirements

- Windows x64 with MSVC, or Linux x64 with GCC 12+ or Clang 14+
  and libstdc++ 12+;
- C++20 and CMake 3.20+ for CMake builds;
- TrueHD CBI input: 48 kHz integer PCM16/PCM24 WAV or RF64 with
  5.1.2, 5.1.4, or 7.1.4 channels. Supply `--input-layout` when the WAV
  has no channel mask.

The main CLI uses native code and does not require FFmpeg or proprietary
encoder DLLs.

## Build

Windows, using an installed Visual C++ toolchain:

```bat
build.bat
out\ddp-atmos.exe --version
out\ddp-atmos.exe --help
```

Or use CMake from a Visual Studio developer shell:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release -LE corpus --output-on-failure
```

With the Visual Studio generator, the CLI is `build/out/Release/ddp-atmos.exe`.

Linux (Debian/Ubuntu):

```sh
sudo apt-get install build-essential cmake
./build.sh
./out/ddp-atmos --version
./out/ddp-atmos --help
ctest --test-dir build-linux -LE corpus --output-on-failure
```

The script copies the CLI to `out/ddp-atmos`; other executables are in
`build-linux/out/`. An optional first argument selects the CLI output path.
Use a separate build directory when changing compilers:

```sh
CXX=clang++ BUILD_DIR=build-linux-clang JOBS=4 ./build.sh
ctest --test-dir build-linux-clang -LE corpus --output-on-failure
```

### GitHub Actions

Every push and pull request builds Windows x64 (MSVC) and Linux x64
(GCC and Clang), runs standalone tests, and checks `--version` and `--help`.
Manual builds are available on the [Build workflow page](https://github.com/almirus/Dolby_enc/actions/workflows/build.yml).

Successful jobs upload archives containing the CLI, README, and license:
`ddp-atmos-windows-x64.zip`, `ddp-atmos-linux-x64-gcc.tar.gz`, and
`ddp-atmos-linux-x64-clang.tar.gz`. Artifacts are retained for 30 days.

Tests labeled `corpus` require local media in `test_file/` and neighboring
directories, which are not included in Git. Remove `-LE corpus` only when
those fixtures are available.

## Quick Start

```sh
# Plain E-AC-3 (the default mode)
ddp-atmos -i surround.wav -o surround.ec3 --bitrate 640000

# Experimental TrueHD Atmos to DD+ Atmos
ddp-atmos -i film.mkv -o film.ec3 --atmos

# Fixed 5.1.2 Atmos bed, wrapped in Matroska
ddp-atmos -i film.mkv -o film_512.mkv --fast-atmos 5.1.2 --mkv

# Lossless TrueHD CBI from a WAV containing height channels
ddp-atmos -i height_714.wav -o height_714.thd --codec truehd --atmos --input-layout 7.1.4

# Export source objects and bed without encoding an output stream
ddp-atmos -i film.mkv --objects-output-dir stems
```

Existing output files are never overwritten. Object export requires a new
destination directory. Use `ddp-atmos --help` for the complete CLI reference.

## License

Distributed under the **PolyForm Noncommercial License 1.0.0**.
See [LICENSE](LICENSE) for the complete terms. Review them before any
commercial use. The license covers original project code; third-party
components retain their respective copyright and license terms.

## Contributing

Bug reports should include the OS, compiler or CLI version, input format,
channel layout, exact command line, and a reproducible sample when redistribution
is allowed. Keep changes focused. Do not commit media fixtures, proprietary
reference binaries, decoder dumps, or generated build products.

## Keywords

**Dolby Atmos · Dolby Digital Plus · E-AC-3 · TrueHD · JOC · OAMD · CBI ·
immersive audio · spatial audio · multichannel audio · audio encoder**

---

# Русский

## Dolby_enc — энкодер Dolby Atmos

Экспериментальная C++20-утилита `ddp-atmos` для кодирования E-AC-3 / Dolby
Digital Plus Atmos и lossless TrueHD CBI с фиксированными высотными каналами.
Проект включает собственные парсеры, декодер TrueHD, пространственный рендерер,
кодирование E-AC-3/JOC и экспорт исходных bed/объектов с метаданными.

Проект независимый и не связан с Dolby Laboratories или LG Electronics.
Товарные знаки используются для описания совместимости и предмета исследования.

## Возможности и ограничения

Обычный E-AC-3 включён по умолчанию. `--atmos` включает экспериментальную
реконструкцию OAMD/JOC; точность объектной сцены и аппаратная совместимость
ещё проверяются. `--fast-atmos` рендерит исходный TrueHD Atmos в фиксированный
bed 5.1.2, 5.1.4 или 7.1.4, заменяя исходные объекты каналами.

`--codec truehd --atmos` кодирует PCM WAV в TrueHD CBI со статичными высотными
каналами. Нужен WAV/RF64 48 кГц с целочисленным PCM16/PCM24 и раскладкой
5.1.2, 5.1.4 или 7.1.4. Без channel mask обязателен `--input-layout`.
Движущиеся объекты и автоматическое создание высоты из обычного surround
этот режим не реализует. Выход TrueHD: raw `.thd` или Matroska с `--mkv`.

Текущая версия — **0.0.1 alfa**.

## Сборка

Windows x64 с MSVC:

```bat
build.bat
out\ddp-atmos.exe --help
```

Альтернативная сборка CMake из developer shell Visual Studio:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release -LE corpus --output-on-failure
```

CLI для генератора Visual Studio: `build/out/Release/ddp-atmos.exe`.

Linux: CMake 3.20+, GCC 12+ или Clang 14+ с libstdc++ 12+:

```sh
sudo apt-get install build-essential cmake
./build.sh
./out/ddp-atmos --help
ctest --test-dir build-linux -LE corpus --output-on-failure
```

Переменные `CXX`, `BUILD_DIR` и `JOBS` задают компилятор, каталог сборки
и число параллельных задач. Первый аргумент `build.sh` меняет путь готового CLI.
Для другого компилятора используйте отдельный `BUILD_DIR`.

Основному CLI не нужны FFmpeg или DLL стороннего энкодера.

## Использование

```sh
ddp-atmos -i surround.wav -o surround.ec3 --bitrate 640000
ddp-atmos -i film.mkv -o film.ec3 --atmos
ddp-atmos -i film.mkv -o film_514.mkv --fast-atmos 5.1.4 --mkv
ddp-atmos -i height_714.wav -o height_714.thd --codec truehd --atmos --input-layout 7.1.4
ddp-atmos -i film.mkv --objects-output-dir stems
```

| Параметр | Назначение |
|---|---|
| `-i`, `--input PATH` | WAV, TrueHD/MLP, E-AC-3, MKV или MP4/M4A; допустимость зависит от режима. |
| `-o`, `--output PATH` | Выходной файл; необязателен при отдельном экспорте объектов. |
| `--codec eac3/truehd` | E-AC-3 по умолчанию или экспериментальный TrueHD CBI. |
| `--input-layout NAME` | Раскладка WAV для TrueHD: 5.1.2, 5.1.4 или 7.1.4. |
| `--audio-track N` | Номер аудиодорожки, начиная с 1. |
| `--duration TIME` | Ограничить длительность, например `10s`, `10m`, `1h2m5s`. |
| `-b`, `--bitrate BPS` | Общий битрейт E-AC-3, по умолчанию 640000 бит/с. |
| `--atmos` | Экспериментальный Atmos; обязателен для TrueHD CBI. |
| `--no-atmos` | Обычный E-AC-3, режим по умолчанию. |
| `--fast-atmos NAME` | TrueHD Atmos → фиксированный bed 5.1.2, 5.1.4 или 7.1.4. |
| `--objects-output-dir PATH` | Экспорт исходного bed/объектов TrueHD/E-AC-3 Atmos в новый каталог. |
| `--mkv` | Упаковать E-AC-3 или TrueHD в Matroska. |
| `--mp4` | Упаковать E-AC-3 в MP4. |
| `--spdif` | Выдать E-AC-3 в IEC 61937 bursts. |
| `--help`, `--version` | Полная справка или версия программы. |

Существующие выходные файлы не перезаписываются. Режимы упаковки
`--mkv`, `--mp4`, `--spdif` взаимоисключающие. У TrueHD нет параметра
битрейта, MP4/SPDIF-выхода или одновременного экспорта объектов.

## GitHub Actions и тесты

При push и pull request проверяются Windows x64 (MSVC) и Linux x64
(GCC/Clang). После сборки запускаются автономные тесты и CLI-проверки.
В Artifacts доступны архивы с программой, README и лицензией на 30 дней.
Ручной запуск — на [странице Build](https://github.com/almirus/Dolby_enc/actions/workflows/build.yml).

Тесты с меткой `corpus` используют локальные медиа из `test_file/` и соседних
каталогов. Они не хранятся в Git и исключены командой `-LE corpus`.
Для запуска полного набора с подготовленными файлами уберите этот фильтр.

## Лицензия и участие

Проект распространяется по **PolyForm Noncommercial License 1.0.0**.
Полный текст — [LICENSE](LICENSE). Перед коммерческим использованием
ознакомьтесь с условиями. Сторонние компоненты сохраняют собственные лицензии.

В отчёте об ошибке укажите ОС, версию, параметры входного потока, раскладку
и точную командную строку. Добавляйте минимальный пример только при наличии
права на его распространение. Не включайте медиафайлы, закрытые эталонные
библиотеки, дампы и результаты сборки в pull request.
