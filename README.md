# dablast

DAB/DAB+ multicast streamer — modelled after dvblast, for digital radio.

Receives a DAB/DAB+ ensemble from an SDR device and streams each radio service as an individual MPEG-TS/UDP multicast stream, one per service. Each stream carries a single audio programme with full DVB PSI tables (PAT, PMT, SDT, EIT) and DLS (now-playing text) updates.

## How it works

```
RTL-SDR (or compatible SDR)
         │
         ▼
   OFDM demodulator
   (frequency sync, FFT, Viterbi)
         │
         ▼
   DAB multiplex decoder
   (FIC → ensemble/service info)
   (MSC → subchannel audio data)
         │
         ▼
   DAB+ superframe filter
   (Reed-Solomon FEC, LATM audio)
         │
         ▼
   MPEG-TS packetiser
   (PAT + PMT + SDT + EIT + audio PES)
         │
         ▼
   UDP multicast  239.x.x.x:port
```

One UDP multicast stream is sent per configured service. VLC, mplayer, ffmpeg, and IPTV clients can tune each address directly.

## Requirements

```bash
sudo apt install \
  build-essential cmake \
  librtlsdr-dev \
  libfftw3-dev \
  libfaad-dev \
  libmpg123-dev
```

Optional:
- `libairspy-dev` — Airspy HF+ support
- `libSoapySDR-dev` — generic SDR support via SoapySDR

## Building

```bash
cmake -B build -S .
cmake --build build -j$(nproc)
```

Optional build flags:

| Flag | Default | Description |
|------|---------|-------------|
| `-DRTLSDR=ON` | ON | RTL-SDR USB dongle support |
| `-DAIRSPY=OFF` | OFF | Airspy HF+ support |
| `-DSOAPYSDR=OFF` | OFF | SoapySDR generic driver |
| `-DKISS_FFT=OFF` | OFF | Use KISS FFT instead of FFTW3 |

## Usage

### Step 1 — Find your frequency and service IDs

Scan a single frequency:

```bash
./dablast -f 202928000 --scan
```

Or scan multiple frequencies in one pass using a frequency list file:

```bash
./dablast -l /etc/dab-Brisbane.freqs --scan
```

Example `/etc/dab-Brisbane.freqs`:
```
197648000 # 8B
202928000 # 9A
204640000 # 9B
206352000 # 9C
208064000 # 9D
```

dablast will scan each frequency in turn, printing results to stdout and
progress/status to stderr. Frequencies with no DAB signal are skipped.

Output example (stderr):
```
Ensemble found. Collecting service list...
Ensemble: DAB+ Brisbane 1 (9A)  EId 0x1006  202928000 Hz
```

Output example (stdout):
```
MMM 80s:202928000:0x12A5
HeartHits:202928000:0x12A7
Inspire Digital:202928000:0x12CD
Coles CBD:202928000:0x1259
GOLD 80s:202928000:0x126D
```

### Step 2 — Create a config file

Each line maps a service to a multicast address:

```
# /etc/dablast-202928000.conf
# Format: MULTICAST_IP:PORT  SERVICE_ID  # optional comment
239.2.2.1:20000  0x12A5  # MMM 80s
239.2.2.2:20000  0x12A7  # HeartHits
239.2.2.3:20000  0x12CD  # Inspire Digital
239.2.2.4:20000  0x1259  # Coles CBD
239.2.2.5:20000  0x126D  # GOLD 80s
```

### Step 3 — Start streaming

```bash
dablast -f 202928000 -c /etc/dablast-202928000.conf
```

### Full option reference

```
Tuning:
  -f FREQ      Centre frequency in Hz  (e.g. 202928000)
  -l FREQFILE  File of frequencies to scan (one per line)

Hardware:
  -a INDEX   Adapter/device index (default: 0)
  -F DRIVER  Input driver: rtl_sdr (default), rtl_tcp, soapysdr
  -g GAIN    RF gain in tenths of dB, or -1 for AGC (default: -1)

Modes:
  --scan     Scan ensemble and print service list, then exit
  -c FILE    Config file for streaming mode

Other:
  -h         Show this help
```

### Multiple ensembles

Run one instance per DAB ensemble (frequency). Each instance uses one SDR device:

```bash
dablast -f 197648000 -a 0 -c /etc/dablast-197648000.conf &
dablast -f 202928000 -a 1 -c /etc/dablast-202928000.conf &
```

## Playing a stream

```bash
# VLC
vlc rtp://@239.2.2.1:20000
```

Or add the multicast addresses to any IPTV playlist (M3U).

### Audio quality

dablast passes audio through from the DAB multiplex **without re-encoding** — the
audio quality is exactly what the broadcaster transmits.

| Service type | Codec | What you should hear |
|---|---|---|
| DAB | MPEG-1 Audio Layer II (MP2) | Stereo, 48 kHz — universally supported |
| DAB+ | HE-AACv2 (SBR + Parametric Stereo) | Stereo, 48 kHz — requires capable decoder |

**DAB+ requires a specific decoder to play at full quality.** Here is what
HE-AACv2 actually does and why it can go wrong:

- The broadcast transmits a **mono AAC core at 24 kHz** — this is the base layer
- **SBR** (Spectral Band Replication) reconstructs the upper frequencies, bringing
  the output up to 48 kHz
- **Parametric Stereo (PS)** reconstructs the stereo image from the mono core,
  giving full left/right stereo

A decoder that handles all three layers correctly will output full 48 kHz stereo.
A decoder that only reads the base layer will output **24 kHz mono** — noticeably
thin and low quality.

**Desktop VLC** works correctly because it uses **FAAD2**, a decoder specifically
built for DAB+ that handles 960-sample AAC frames and the full HE-AACv2 chain.

Many other players (Android apps, hardware IPTV boxes, some media servers) use
hardware AAC decoders or general-purpose software decoders that do not support
the 960-sample frame size used by DAB+. These will fall back to the 24 kHz mono
base layer regardless of how the stream is signalled. This is a **decoder
limitation**, not a problem with the stream itself — the stream content is correct.

There is no workaround short of transcoding the audio on the receiving end, to decode and re-encode to a standard 1024-sample AAC stream.

## Supported hardware

- **RTL-SDR** — any RTL2832U-based USB dongle (default, tested)

The following drivers are included from the welle.io backend and should work
in principle, but have not been tested with dablast:

- **Airspy HF+** — compile with `-DAIRSPY=ON`
- **SoapySDR** — any SoapySDR-supported device, compile with `-DSOAPYSDR=ON`
- **rtl_tcp** — network-attached RTL-SDR via rtl_tcp server, use `-F rtl_tcp`
- **Raw IQ file** — for testing, use `-F rawfile`

## License

Copyright (C) 2026 Paul Stanley <paul.stanley@tutanota.com>

GNU General Public License v3.0 or later — see [LICENSE.md](LICENSE.md).

This project incorporates code from [welle.io](https://github.com/AlbrechtL/welle.io),
[SDR-J](http://www.sdr-j.tk), [DABlin](https://github.com/Opendigitalradio/dablin),
and third-party libraries. See LICENSE.md for details.
