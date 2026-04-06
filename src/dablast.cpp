/*
 * dablast - DAB/DAB+ multicast streamer
 * Modelled after dvblast, for DAB/DAB+ radio.
 *
 * Copyright (C) 2026  Paul Stanley <paul.stanley@tutanota.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *
 * Usage:
 *   dablast -f FREQUENCY --scan          # scan and print services
 *   dablast -f FREQUENCY -c CONFIG       # stream services as MPEG-TS/UDP multicast
 */

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <getopt.h>

#include "backend/radio-receiver.h"
#include "backend/radio-controller.h"
#include "backend/dab-constants.h"
#include "backend/subchannel_sink.h"
#include "input/input_factory.h"
#include "various/channels.h"
#include "sink_registry.h"

#ifdef HAVE_RTLSDR
#include "input/rtl_sdr.h"
#endif

using namespace std;
using namespace chrono;

// ─────────────────────────────────────────────────────────────────────────────
// Signal handling
// ─────────────────────────────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;
static void sigint_handler(int) { g_running = 0; }

// ─────────────────────────────────────────────────────────────────────────────
// MPEG-2 CRC32 (polynomial 0x04C11DB7, MSB-first, initial value 0xFFFFFFFF)
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t crc32_mpeg(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
    }
    return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Write PTS (5 bytes, 90 kHz clock)
// ─────────────────────────────────────────────────────────────────────────────
static void write_pts(uint8_t* b, uint64_t pts) {
    b[0] = 0x21 | (uint8_t)((pts >> 29) & 0x0E);
    b[1] = (uint8_t)((pts >> 22) & 0xFF);
    b[2] = 0x01 | (uint8_t)((pts >> 14) & 0xFE);
    b[3] = (uint8_t)((pts >>  7) & 0xFF);
    b[4] = 0x01 | (uint8_t)((pts <<  1) & 0xFE);
}

// ─────────────────────────────────────────────────────────────────────────────
// TS packet writer – writes exactly 188 bytes into buf
// ─────────────────────────────────────────────────────────────────────────────
static void ts_write(uint8_t* buf, uint16_t pid, uint8_t& cc, bool pusi,
                     const uint8_t* payload, size_t plen)
{
    // payload_len <= 184; remaining bytes become adaptation-field stuffing
    size_t stuff = 184 - plen;
    buf[0] = 0x47;
    buf[1] = (pusi ? 0x40 : 0x00) | (uint8_t)((pid >> 8) & 0x1F);
    buf[2] = (uint8_t)(pid & 0xFF);
    if (stuff == 0) {
        buf[3] = 0x10 | (cc & 0x0F);
        if (payload) memcpy(buf + 4, payload, plen);
    } else {
        buf[3] = 0x30 | (cc & 0x0F);   // adaptation + payload
        buf[4] = (uint8_t)(stuff - 1);  // adaptation_field_length
        if (stuff >= 2) { buf[5] = 0x00; }               // flags
        if (stuff >  2) { memset(buf + 6, 0xFF, stuff - 2); }  // stuffing
        if (payload && plen > 0) memcpy(buf + 4 + stuff, payload, plen);
    }
    cc = (cc + 1) & 0x0F;
}

// ─────────────────────────────────────────────────────────────────────────────
// PSI packet helper – wraps one section in a single TS packet
// (PUSI=1, pointer_field=0x00, 0xFF padding)
// ─────────────────────────────────────────────────────────────────────────────
static void ts_psi(uint8_t* buf, uint16_t pid, uint8_t& cc,
                   const vector<uint8_t>& sec)
{
    uint8_t pl[184];
    pl[0] = 0x00;  // pointer_field
    size_t copy = min(sec.size(), (size_t)183);
    memcpy(pl + 1, sec.data(), copy);
    if (1 + copy < 184) memset(pl + 1 + copy, 0xFF, 183 - copy);
    ts_write(buf, pid, cc, true, pl, 184);
}

// ─────────────────────────────────────────────────────────────────────────────
// Build PAT section
// ─────────────────────────────────────────────────────────────────────────────
static vector<uint8_t> build_pat(uint16_t pmt_pid) {
    vector<uint8_t> s;
    s.push_back(0x00);  // table_id
    s.push_back(0xB0); s.push_back(0x0D);   // section_length=13
    s.push_back(0x00); s.push_back(0x01);   // transport_stream_id=1
    s.push_back(0xC1);                       // version=0, current_next=1
    s.push_back(0x00); s.push_back(0x00);   // section/last_section_number
    // program 1 → PMT PID
    s.push_back(0x00); s.push_back(0x01);
    s.push_back(0xE0 | (uint8_t)(pmt_pid >> 8));
    s.push_back((uint8_t)(pmt_pid & 0xFF));
    uint32_t crc = crc32_mpeg(s.data(), s.size());
    s.push_back((crc>>24)&0xFF); s.push_back((crc>>16)&0xFF);
    s.push_back((crc>> 8)&0xFF); s.push_back(crc&0xFF);
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Build PMT section
// ─────────────────────────────────────────────────────────────────────────────
static vector<uint8_t> build_pmt(uint16_t prog, uint16_t pcr_pid,
                                   uint16_t audio_pid, uint8_t stream_type) {
    // ES descriptors: for DAB+ (stream_type=0x11) add the DVB AAC descriptor
    // (ETSI EN 300 468 tag 0x7C) to declare HE-AAC v2 profile.  Without it
    // players see LATM and assume AAC-LC, playing only the core: mono 24 kHz.
    vector<uint8_t> es_desc;
    if (stream_type == 0x11) {
        es_desc.push_back(0x7C);  // AAC_descriptor tag
        es_desc.push_back(0x01);  // length = 1
        es_desc.push_back(0x58);  // profile_and_level: HE-AAC v2 Level 2
    }

    // section_length = 2+1+1+1+2+2+(5+es_desc.size())+4
    uint16_t sec_len = (uint16_t)(18 + es_desc.size());
    vector<uint8_t> s;
    s.push_back(0x02);  // table_id
    s.push_back(0xB0 | (uint8_t)(sec_len >> 8));
    s.push_back((uint8_t)(sec_len & 0xFF));
    s.push_back((uint8_t)(prog >> 8)); s.push_back((uint8_t)(prog & 0xFF));
    s.push_back(0xC1);
    s.push_back(0x00); s.push_back(0x00);
    s.push_back(0xE0 | (uint8_t)(pcr_pid >> 8));
    s.push_back((uint8_t)(pcr_pid & 0xFF));
    s.push_back(0xF0); s.push_back(0x00);  // program_info_length=0
    // stream entry
    s.push_back(stream_type);
    s.push_back(0xE0 | (uint8_t)(audio_pid >> 8));
    s.push_back((uint8_t)(audio_pid & 0xFF));
    s.push_back(0xF0); s.push_back((uint8_t)es_desc.size());  // ES_info_length
    s.insert(s.end(), es_desc.begin(), es_desc.end());
    uint32_t crc = crc32_mpeg(s.data(), s.size());
    s.push_back((crc>>24)&0xFF); s.push_back((crc>>16)&0xFF);
    s.push_back((crc>> 8)&0xFF); s.push_back(crc&0xFF);
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// DVB SI table PIDs and identifiers
// ─────────────────────────────────────────────────────────────────────────────
static const uint16_t PID_SDT   = 0x0011;
static const uint16_t PID_EIT   = 0x0012;
static const uint16_t DVB_TS_ID = 0x0001;   // transport_stream_id
static const uint16_t DVB_NW_ID = 0xFF01;   // original_network_id (private range)
static const uint16_t DVB_PROG  = 0x0001;   // program number (matches PAT)

// Encode current UTC as 5-byte MJD + BCD hhmmss for EIT start_time
static void encode_mjd_utc(uint8_t* b)
{
    time_t now = time(nullptr);
    struct tm* u = gmtime(&now);
    int Y = u->tm_year, M = u->tm_mon + 1, D = u->tm_mday;
    int y = (M <= 2) ? Y - 1 : Y;
    int m = (M <= 2) ? M + 12 : M;
    int mjd = 14956 + D + (int)(y * 365.25) + (int)((m + 1) * 30.6001);
    b[0] = (uint8_t)(mjd >> 8);  b[1] = (uint8_t)(mjd & 0xFF);
    auto bcd = [](int v) -> uint8_t { return (uint8_t)(((v/10)<<4)|(v%10)); };
    b[2] = bcd(u->tm_hour); b[3] = bcd(u->tm_min); b[4] = bcd(u->tm_sec);
}

// Build SDT (Service Description Table, table_id=0x42) – one service entry.
// service_name is the human-readable label (e.g. "Hot Tomato 102.9").
static vector<uint8_t> build_sdt(const string& service_name)
{
    // DVB text: 0x15 prefix = UTF-8
    string pname = "\x15" "dablast";
    string sname = "\x15" + service_name;

    // Service descriptor tag=0x48: service_type(1)+prov_len(1)+prov+name_len(1)+name
    vector<uint8_t> sdesc;
    sdesc.push_back(0x48);
    sdesc.push_back((uint8_t)(1 + 1 + pname.size() + 1 + sname.size()));
    sdesc.push_back(0x02);  // digital radio sound service
    sdesc.push_back((uint8_t)pname.size());
    sdesc.insert(sdesc.end(), pname.begin(), pname.end());
    sdesc.push_back((uint8_t)sname.size());
    sdesc.insert(sdesc.end(), sname.begin(), sname.end());

    uint16_t dlen = (uint16_t)sdesc.size();
    // section_length covers from transport_stream_id to CRC:
    //   ts_id(2)+ver(1)+sec(1)+last(1)+orig_nw(2)+reserved(1) = 8
    //   + service_id(2)+flags(1)+desc_loop_len(2)+descs + CRC(4)
    uint16_t sec_len = (uint16_t)(8 + 2 + 1 + 2 + dlen + 4);

    vector<uint8_t> s;
    s.push_back(0x42);  // table_id: SDT actual TS
    s.push_back((uint8_t)(0xF0 | (sec_len >> 8)));
    s.push_back((uint8_t)(sec_len & 0xFF));
    s.push_back((uint8_t)(DVB_TS_ID >> 8)); s.push_back((uint8_t)(DVB_TS_ID & 0xFF));
    s.push_back(0xC1);  // reserved(2)=11 version(5)=0 current_next=1
    s.push_back(0x00);  // section_number
    s.push_back(0x00);  // last_section_number
    s.push_back((uint8_t)(DVB_NW_ID >> 8)); s.push_back((uint8_t)(DVB_NW_ID & 0xFF));
    s.push_back(0xFF);  // reserved_future_use
    // service loop entry
    s.push_back((uint8_t)(DVB_PROG >> 8)); s.push_back((uint8_t)(DVB_PROG & 0xFF));
    // reserved(6)=1, EIT_schedule=0, EIT_present_following=1 → 0b11111101
    s.push_back(0xFD);
    // running_status=4(running), free_CA_mode=0, desc_loop_len(12 bits)
    s.push_back((uint8_t)(0x80 | (dlen >> 8)));
    s.push_back((uint8_t)(dlen & 0xFF));
    s.insert(s.end(), sdesc.begin(), sdesc.end());
    uint32_t crc = crc32_mpeg(s.data(), s.size());
    s.push_back((crc>>24)&0xFF); s.push_back((crc>>16)&0xFF);
    s.push_back((crc>>8)&0xFF);  s.push_back(crc&0xFF);
    return s;
}

// Build EIT present/following section 0 (table_id=0x4E) for one event.
// event_text is the DLS now-playing string.
// version increments (mod 32) each time the content changes.
static vector<uint8_t> build_eit(uint8_t version, const string& event_text)
{
    // short_event_descriptor tag=0x4D: lang(3)+name_len(1)+name+text_len(1)
    string ename = "\x15" + event_text;  // UTF-8 prefix
    vector<uint8_t> evd;
    evd.push_back(0x4D);
    evd.push_back((uint8_t)(3 + 1 + ename.size() + 1));
    evd.push_back('e'); evd.push_back('n'); evd.push_back('g');
    evd.push_back((uint8_t)ename.size());
    evd.insert(evd.end(), ename.begin(), ename.end());
    evd.push_back(0x00);  // text_length = 0

    uint16_t dlen = (uint16_t)evd.size();
    // section_length: fixed EIT header after sec_len = 11 bytes
    //   service_id(2)+ver(1)+sec(1)+last(1)+ts_id(2)+orig_nw(2)+seg_last(1)+last_tbl(1)
    // + event: event_id(2)+start(5)+dur(3)+status+dlen(2)+descs + CRC(4)
    uint16_t sec_len = (uint16_t)(11 + 2 + 5 + 3 + 2 + dlen + 4);

    vector<uint8_t> s;
    s.push_back(0x4E);  // table_id: EIT p/f actual TS
    s.push_back((uint8_t)(0xF0 | (sec_len >> 8)));
    s.push_back((uint8_t)(sec_len & 0xFF));
    s.push_back((uint8_t)(DVB_PROG >> 8)); s.push_back((uint8_t)(DVB_PROG & 0xFF));
    s.push_back((uint8_t)(0xC0 | ((version & 0x1F) << 1) | 0x01));  // ver + current_next
    s.push_back(0x00);  // section_number = 0 (present event)
    s.push_back(0x00);  // last_section_number
    s.push_back((uint8_t)(DVB_TS_ID >> 8)); s.push_back((uint8_t)(DVB_TS_ID & 0xFF));
    s.push_back((uint8_t)(DVB_NW_ID >> 8)); s.push_back((uint8_t)(DVB_NW_ID & 0xFF));
    s.push_back(0x00);  // segment_last_section_number
    s.push_back(0x4E);  // last_table_id
    // event entry
    s.push_back(0x00); s.push_back(0x01);  // event_id = 1
    uint8_t mjd[5]; encode_mjd_utc(mjd);
    s.insert(s.end(), mjd, mjd + 5);       // start_time (current UTC)
    s.push_back(0x00); s.push_back(0x00); s.push_back(0x00);  // duration unknown
    // running_status=4, free_CA_mode=0, desc_loop_len
    s.push_back((uint8_t)(0x80 | (dlen >> 8)));
    s.push_back((uint8_t)(dlen & 0xFF));
    s.insert(s.end(), evd.begin(), evd.end());
    uint32_t crc = crc32_mpeg(s.data(), s.size());
    s.push_back((crc>>24)&0xFF); s.push_back((crc>>16)&0xFF);
    s.push_back((crc>>8)&0xFF);  s.push_back(crc&0xFF);
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Config file parser
// ─────────────────────────────────────────────────────────────────────────────
struct StreamConfig {
    string   mcast;
    uint16_t port;
    uint32_t sid;
};

// Parse lines like:  239.1.0.1:20000  0x1234  # Name
static vector<StreamConfig> parse_config(const string& path) {
    vector<StreamConfig> cfgs;
    ifstream f(path);
    if (!f) { cerr << "Cannot open config: " << path << "\n"; return cfgs; }
    string line;
    while (getline(f, line)) {
        // strip comment
        auto pos = line.find('#');
        if (pos != string::npos) line = line.substr(0, pos);
        // trim trailing whitespace
        while (!line.empty() && isspace((unsigned char)line.back())) line.pop_back();
        if (line.empty()) continue;
        istringstream ss(line);
        string addr_port, sid_str;
        if (!(ss >> addr_port >> sid_str)) continue;
        // split addr:port
        auto colon = addr_port.rfind(':');
        if (colon == string::npos) continue;
        StreamConfig c;
        c.mcast = addr_port.substr(0, colon);
        try { c.port = (uint16_t)stoul(addr_port.substr(colon+1)); }
        catch(...) { continue; }
        try { c.sid = (uint32_t)stoul(sid_str, nullptr, 0); }
        catch(...) { continue; }
        cfgs.push_back(c);
    }
    return cfgs;
}

// ─────────────────────────────────────────────────────────────────────────────
// Minimal controller – collects ensemble/service state from FIC callbacks
// ─────────────────────────────────────────────────────────────────────────────
class DablastController : public RadioControllerInterface {
public:
    atomic<bool> synced{false};
    atomic<bool> signal_lost{false};
    uint16_t     ensemble_id    = 0;
    string       ensemble_label;

    mutex              cv_mut;
    condition_variable cv;

    void onSNR(float) override {}
    void onFrequencyCorrectorChange(int, int) override {}

    void onSyncChange(char isSync) override {
        synced = (isSync != 0);
        cv.notify_all();
    }

    void onSignalPresence(bool isSignal) override {
        if (!isSignal) signal_lost = true;
    }

    void onServiceDetected(uint32_t) override {
        cv.notify_all();
    }

    void onNewEnsemble(uint16_t eId) override {
        ensemble_id = eId;
        cv.notify_all();
    }

    void onSetEnsembleLabel(DabLabel& label) override {
        ensemble_label = label.utf8_label();
        cv.notify_all();
    }

    void onDateTimeUpdate(const dab_date_time_t&) override {}
    void onFIBDecodeSuccess(bool, const uint8_t*) override {}
    void onNewImpulseResponse(vector<float>&&) override {}
    void onNewNullSymbol(vector<DSPCOMPLEX>&&) override {}
    void onConstellationPoints(vector<DSPCOMPLEX>&&) override {}
    void onTIIMeasurement(tii_measurement_t&&) override {}

    void onMessage(message_level_t level, const string& text,
                   const string& text2) override
    {
        if (level == message_level_t::Error)
            cerr << "[error] " << text << text2 << "\n";
    }

    void onInputFailure() override {
        signal_lost = true;
        cv.notify_all();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// ServiceStream – one per multicast group; wraps audio frames in MPEG-TS/UDP
// ─────────────────────────────────────────────────────────────────────────────
static const uint16_t PID_PAT   = 0x0000;
static const uint16_t PID_PMT   = 0x1000;
static const uint16_t PID_AUDIO = 0x0101;
static const int TS_PER_DGRAM   = 7;  // 7*188=1316 bytes per UDP datagram

class ServiceStream : public UntouchedStreamConsumer, public ProgrammeHandlerInterface {
public:
    ServiceStream(const string& mcast_ip, uint16_t port, uint32_t sid,
                  uint8_t stream_type, const string& service_name)
        : sid_(sid), stream_type_(stream_type), service_name_(service_name)
    {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) { perror("socket"); return; }
        int ttl = 32;
        setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
        memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port   = htons(port);
        if (inet_pton(AF_INET, mcast_ip.c_str(), &dest_.sin_addr) != 1) {
            cerr << "Invalid address: " << mcast_ip << "\n";
            close(sock_); sock_ = -1;
        }
        rtp_ssrc_ = sid_;  // unique per service

        // Pre-build sections that never change
        pat_vec_ = build_pat(PID_PMT);
        pmt_vec_ = build_pmt(DVB_PROG, PID_AUDIO, PID_AUDIO, stream_type_);
        sdt_vec_ = build_sdt(service_name_);
        eit_vec_ = build_eit(0, service_name_);  // placeholder until DLS arrives
    }

    ~ServiceStream() { if (sock_ >= 0) close(sock_); }

    bool is_open() const { return sock_ >= 0; }
    uint32_t sid() const { return sid_; }

    // UntouchedStreamConsumer – called from decoder thread with LATM-wrapped AAC
    void ProcessUntouchedStream(const uint8_t* data, size_t len,
                                 size_t duration_ms) override
    {
        if (sock_ < 0) return;

        // Snapshot PTS for RTP timestamp before advancing the clock
        rtp_ts_ = (uint32_t)pts_;

        // Send updated EIT immediately when DLS (now-playing text) changes
        if (dls_changed_) {
            send_eit_update();
            dls_changed_ = false;
        }

        // PAT + PMT + SDT + EIT every ~1 s (once per 50 frames at 20 ms each)
        if (frame_cnt_++ % 50 == 0)
            send_pat_pmt();

        send_pes(data, len, pts_);
        pts_ += (uint64_t)duration_ms * 90;  // advance 90 kHz clock

        flush_buf();  // send whatever remains in the 7-packet buffer
    }

    // ProgrammeHandlerInterface – no-ops; we use the USC path, not PCM
    void onFrameErrors(int) override {}
    void onNewAudio(vector<int16_t>&&, int, const string&) override {}
    void onRsErrors(bool, int) override {}
    void onAacErrors(int) override {}
    void onNewDynamicLabel(const string& label) override {
        current_dls_  = label;
        eit_version_  = (eit_version_ + 1) & 0x1F;
        dls_changed_  = true;
    }
    void onMOT(const mot_file_t&) override {}
    void onPADLengthError(size_t, size_t) override {}

private:
    uint32_t sid_;
    uint8_t  stream_type_;
    string   service_name_;
    int      sock_ = -1;
    struct   sockaddr_in dest_;

    uint8_t  cc_pat_    = 0;
    uint8_t  cc_pmt_    = 0;
    uint8_t  cc_sdt_    = 0;
    uint8_t  cc_eit_    = 0;
    uint8_t  cc_audio_  = 0;
    uint64_t pts_       = 0;
    uint32_t frame_cnt_ = 0;

    // DLS (Dynamic Label / now-playing) state
    string   current_dls_;
    uint8_t  eit_version_ = 0;
    bool     dls_changed_ = false;

    // Pre-built section bytes
    vector<uint8_t> pat_vec_;
    vector<uint8_t> pmt_vec_;
    vector<uint8_t> sdt_vec_;  // static: service name never changes
    vector<uint8_t> eit_vec_;  // rebuilt on each DLS change

    // RTP state (RFC 2250 – MPEG-TS over RTP, PT=33)
    uint16_t rtp_seq_  = 0;
    uint32_t rtp_ssrc_ = 0;   // set from sid_ in constructor
    uint32_t rtp_ts_   = 0;   // 90 kHz, captured at start of each ProcessUntouchedStream

    uint8_t  buf_[TS_PER_DGRAM * 188];
    int      buf_n_    = 0;   // number of TS packets currently in buf_

    // ── push one 188-byte TS packet; flush when 7 are accumulated ──────────
    void push_pkt(uint16_t pid, uint8_t& cc, bool pusi,
                  const uint8_t* pl, size_t plen)
    {
        ts_write(buf_ + buf_n_ * 188, pid, cc, pusi, pl, plen);
        if (++buf_n_ == TS_PER_DGRAM) send_dgram();
    }

    // ── flush remaining packets (padded with null packets) ──────────────────
    void flush_buf() {
        if (buf_n_ == 0) return;
        // pad with null TS packets (PID 0x1FFF)
        static const uint8_t null_hdr[4] = {0x47, 0x1F, 0xFF, 0x10};
        while (buf_n_ < TS_PER_DGRAM) {
            uint8_t* p = buf_ + buf_n_ * 188;
            memcpy(p, null_hdr, 4);
            memset(p + 4, 0xFF, 184);
            buf_n_++;
        }
        send_dgram();
    }

    void send_dgram() {
        // RFC 2250: 12-byte RTP header + 7×188 MPEG-TS = 1328 bytes (same as dvblast)
        uint8_t dgram[12 + TS_PER_DGRAM * 188];
        dgram[0]  = 0x80;                          // V=2, P=0, X=0, CC=0
        dgram[1]  = 0x21;                          // M=0, PT=33 (MPEG-TS)
        dgram[2]  = (uint8_t)(rtp_seq_ >> 8);
        dgram[3]  = (uint8_t)(rtp_seq_ & 0xFF);
        rtp_seq_++;
        dgram[4]  = (uint8_t)(rtp_ts_ >> 24);     // 90 kHz timestamp
        dgram[5]  = (uint8_t)(rtp_ts_ >> 16);
        dgram[6]  = (uint8_t)(rtp_ts_ >> 8);
        dgram[7]  = (uint8_t)(rtp_ts_ & 0xFF);
        dgram[8]  = (uint8_t)(rtp_ssrc_ >> 24);
        dgram[9]  = (uint8_t)(rtp_ssrc_ >> 16);
        dgram[10] = (uint8_t)(rtp_ssrc_ >> 8);
        dgram[11] = (uint8_t)(rtp_ssrc_ & 0xFF);
        memcpy(dgram + 12, buf_, TS_PER_DGRAM * 188);
        sendto(sock_, dgram, sizeof(dgram), 0,
               (struct sockaddr*)&dest_, sizeof(dest_));
        buf_n_ = 0;
    }

    // ── send PAT + PMT + SDT + EIT ───────────────────────────────────────────
    void send_pat_pmt() {
        ts_psi(buf_ + buf_n_ * 188, PID_PAT, cc_pat_, pat_vec_);
        if (++buf_n_ == TS_PER_DGRAM) send_dgram();

        ts_psi(buf_ + buf_n_ * 188, PID_PMT, cc_pmt_, pmt_vec_);
        if (++buf_n_ == TS_PER_DGRAM) send_dgram();

        ts_psi(buf_ + buf_n_ * 188, PID_SDT, cc_sdt_, sdt_vec_);
        if (++buf_n_ == TS_PER_DGRAM) send_dgram();

        ts_psi(buf_ + buf_n_ * 188, PID_EIT, cc_eit_, eit_vec_);
        if (++buf_n_ == TS_PER_DGRAM) send_dgram();
    }

    // ── rebuild and send EIT immediately on DLS change ────────────────────────
    void send_eit_update() {
        eit_vec_ = build_eit(eit_version_, current_dls_);
        ts_psi(buf_ + buf_n_ * 188, PID_EIT, cc_eit_, eit_vec_);
        if (++buf_n_ == TS_PER_DGRAM) send_dgram();
    }

    // ── wrap LATM data in PES and split into TS packets ─────────────────────
    void send_pes(const uint8_t* data, size_t dlen, uint64_t pts)
    {
        // Build 14-byte PES header
        uint8_t pes_hdr[14];
        {
            uint16_t pes_body = (uint16_t)(3 + 5 + dlen);  // optional_pes_hdr(3)+pts(5)+payload
            pes_hdr[0] = 0x00; pes_hdr[1] = 0x00; pes_hdr[2] = 0x01;
            pes_hdr[3] = 0xC0;
            pes_hdr[4] = (uint8_t)(pes_body >> 8);
            pes_hdr[5] = (uint8_t)(pes_body & 0xFF);
            pes_hdr[6] = 0x80;   // marker bits
            pes_hdr[7] = 0x80;   // PTS_DTS_flags = PTS only
            pes_hdr[8] = 0x05;   // PES_header_data_length
            write_pts(pes_hdr + 9, pts);
        }

        // First TS packet: adaptation field carrying PCR (8 bytes = 1 afl + 7 content)
        //   → available payload: 184 - 8 = 176 bytes
        // PCR encoding: base = pts (33 bits, 90 kHz), extension = 0
        const size_t avail_first = 176;
        const size_t pes_total   = sizeof(pes_hdr) + dlen;
        const size_t first_copy  = min(pes_total, avail_first);
        const size_t extra_stuff = avail_first - first_copy;  // pad AF if PES fits in < 176 B

        {
            uint8_t pkt[188];
            pkt[0] = 0x47;
            pkt[1] = 0x40 | (uint8_t)((PID_AUDIO >> 8) & 0x1F);  // PUSI=1
            pkt[2] = (uint8_t)(PID_AUDIO & 0xFF);
            pkt[3] = 0x30 | (cc_audio_ & 0x0F);  // adaptation + payload
            cc_audio_ = (cc_audio_ + 1) & 0x0F;

            // Adaptation field
            pkt[4] = (uint8_t)(7 + extra_stuff);  // adaptation_field_length
            pkt[5] = 0x10;                          // PCR_flag = 1
            pkt[6]  = (uint8_t)(pts >> 25);         // PCR_base[32..25]
            pkt[7]  = (uint8_t)(pts >> 17);         // PCR_base[24..17]
            pkt[8]  = (uint8_t)(pts >> 9);          // PCR_base[16..9]
            pkt[9]  = (uint8_t)(pts >> 1);          // PCR_base[8..1]
            pkt[10] = (uint8_t)((pts & 1) << 7) | 0x7E; // PCR_base[0]+reserved+ext[8]=0
            pkt[11] = 0x00;                          // PCR_extension[7..0] = 0
            if (extra_stuff > 0) memset(pkt + 12, 0xFF, extra_stuff);

            // Payload: PES header then data
            uint8_t* pl = pkt + 12 + extra_stuff;
            size_t hdr_n = min(sizeof(pes_hdr), first_copy);
            memcpy(pl, pes_hdr, hdr_n);
            if (first_copy > sizeof(pes_hdr))
                memcpy(pl + sizeof(pes_hdr), data, first_copy - sizeof(pes_hdr));

            memcpy(buf_ + buf_n_ * 188, pkt, 188);
            if (++buf_n_ == TS_PER_DGRAM) send_dgram();
        }

        // Continuation TS packets (no PCR, just payload ± stuffing via push_pkt)
        size_t offset = (first_copy > sizeof(pes_hdr)) ? (first_copy - sizeof(pes_hdr)) : 0;
        while (offset < dlen) {
            size_t chunk = min((size_t)184, dlen - offset);
            push_pkt(PID_AUDIO, cc_audio_, false, data + offset, chunk);
            offset += chunk;
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Usage
// ─────────────────────────────────────────────────────────────────────────────
static void usage(const char* prog)
{
    cerr <<
        "dablast - DAB multicast streamer\n"
        "\n"
        "Usage:\n"
        "  " << prog << " -f FREQ [options] --scan\n"
        "  " << prog << " -l FREQFILE [options] --scan\n"
        "  " << prog << " -f FREQ [options] -c CONFIG\n"
        "\n"
        "Tuning:\n"
        "  -f FREQ      Centre frequency in Hz\n"
        "               e.g. 202928000  or  174928000\n"
        "  -l FREQFILE  File of frequencies to scan (one per line, # comments ok)\n"
        "\n"
        "Device:\n"
        "  -a NUM     RTL-SDR adapter index (default: 0)\n"
        "  -F DRIVER  Input driver: rtl_sdr (default), rtl_tcp, soapysdr\n"
        "  -g GAIN    RF gain in tenths of dB, or -1 for AGC (default: -1)\n"
        "\n"
        "Modes:\n"
        "  --scan     Scan ensemble and print service list, then exit\n"
        "  -c FILE    Config file for streaming mode\n"
        "\n"
        "Other:\n"
        "  -h         Show this help\n"
        "\n"
        "Examples:\n"
        "  dablast -f 202928000 --scan\n"
        "  dablast -l /etc/dab-brisbane.freqs --scan\n"
        "  dablast -f 202928000 -a 1 -g 496 --scan\n"
        "  dablast -f 202928000 -a 0 -c /etc/dablast-202928000.conf\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Scan mode – block until ensemble is fully read, then dump service table
// ─────────────────────────────────────────────────────────────────────────────
static int do_scan(RadioReceiver& rx, DablastController& ctrl, long freq_hz)
{
    // 1. Wait for OFDM sync (up to 30 s — PLL may need time to settle)
    {
        unique_lock<mutex> lk(ctrl.cv_mut);
        bool ok = ctrl.cv.wait_for(lk, seconds(30),
                [&]{ return ctrl.synced.load() || ctrl.signal_lost.load(); });
        if (!ok || ctrl.signal_lost) {
            cerr << "No DAB signal on " << freq_hz << " Hz – skipping.\n";
            return 1;
        }
    }

    cerr << "Signal acquired. Waiting for ensemble information...\n";

    // 2. Wait for ensemble label (up to 20 s after sync)
    {
        unique_lock<mutex> lk(ctrl.cv_mut);
        ctrl.cv.wait_for(lk, seconds(20),
                [&]{ return !ctrl.ensemble_label.empty(); });
    }

    if (ctrl.ensemble_label.empty()) {
        cerr << "Sync'd but no ensemble label received. Try again or check frequency.\n";
        return 1;
    }

    // 3. Let the service list settle – FIG 0/2 repeats every ~1 s but some
    //    broadcasters are slow; 5 s is a safe wait.
    cerr << "Ensemble found. Collecting service list...\n";
    this_thread::sleep_for(seconds(5));

    auto services = rx.getServiceList();
    if (services.empty()) {
        cerr << "No services found.\n";
        return 1;
    }

    // 4. Log ensemble info to stderr
    Channels channels;
    string chan_name = channels.getChannelForFrequency((int)freq_hz);
    cerr << "Ensemble: " << ctrl.ensemble_label;
    if (!chan_name.empty()) cerr << " (" << chan_name << ")";
    cerr << "  EId 0x" << hex << uppercase << ctrl.ensemble_id << dec
         << "  " << freq_hz << " Hz\n";

    // 5. Print one line per service to stdout: name:frequency:sid
    for (const auto& svc : services) {
        auto comps = rx.getComponents(svc);
        for (const auto& sc : comps) {
            if (sc.transportMode() != TransportMode::Audio)
                continue;
            if (!rx.getSubchannel(sc).valid())
                continue;
            string label = svc.serviceLabel.utf8_label();
            while (!label.empty() && isspace((unsigned char)label.back()))
                label.pop_back();
            cout << label << ":" << freq_hz
                 << ":0x" << hex << uppercase << svc.serviceId << dec << "\n";
            break;  // first audio component only
        }
    }

    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stream mode – decode services and multicast as MPEG-TS/UDP
// ─────────────────────────────────────────────────────────────────────────────
static int do_stream(RadioReceiver& rx, DablastController& ctrl,
                     long freq_hz, const string& config_path)
{
    // 1. Parse config file
    auto cfgs = parse_config(config_path);
    if (cfgs.empty()) {
        cerr << "No valid entries in config file: " << config_path << "\n";
        return 1;
    }

    // 2. Wait for OFDM sync – no timeout; Ctrl+C exits
    cerr << "Waiting for DAB signal on " << freq_hz << " Hz...\n";
    {
        unique_lock<mutex> lk(ctrl.cv_mut);
        ctrl.cv.wait(lk, [&]{
            return ctrl.synced.load() || ctrl.signal_lost.load() || !g_running;
        });
        if (!g_running) return 0;
    }

    cerr << "Signal acquired. Waiting for ensemble...\n";

    // 3. Wait for ensemble label – no timeout; Ctrl+C exits
    {
        unique_lock<mutex> lk(ctrl.cv_mut);
        ctrl.cv.wait(lk, [&]{
            return !ctrl.ensemble_label.empty() || ctrl.signal_lost.load() || !g_running;
        });
        if (!g_running) return 0;
        if (ctrl.ensemble_label.empty()) {
            cerr << "Signal lost before ensemble was received.\n";
            return 1;
        }
    }

    // 4. Let the service list settle (5 s)
    cerr << "Ensemble: " << ctrl.ensemble_label << ". Collecting services...\n";
    this_thread::sleep_for(seconds(5));

    auto services = rx.getServiceList();

    // 5. For each config entry, set up a ServiceStream
    struct StreamEntry {
        unique_ptr<ServiceStream> stream;
        SubchannelSink* sink;
        uint32_t sid;
    };
    vector<StreamEntry> entries;

    for (auto& cfg : cfgs) {
        // Find service
        const Service* svc_ptr = nullptr;
        for (const auto& s : services) {
            if (s.serviceId == cfg.sid) { svc_ptr = &s; break; }
        }
        if (!svc_ptr) {
            cerr << "SID 0x" << hex << cfg.sid << dec
                  << " not found in ensemble – skipping.\n";
            continue;
        }
        const Service& svc = *svc_ptr;

        // Determine audio type for stream_type in PMT
        uint8_t stream_type = 0x11;  // default: AAC LATM (DAB+)
        auto comps = rx.getComponents(svc);
        for (auto& sc : comps) {
            if (sc.transportMode() == TransportMode::Audio) {
                stream_type = (sc.audioType() == AudioServiceComponentType::DABPlus)
                              ? 0x11   // ISO 14496-3 LATM (DAB+/HE-AAC)
                              : 0x03;  // MPEG-1 Audio Layer 2 (DAB/MP2)
                break;
            }
        }

        string label = svc.serviceLabel.utf8_label();
        while (!label.empty() && isspace((unsigned char)label.back()))
            label.pop_back();
        auto ss = make_unique<ServiceStream>(cfg.mcast, cfg.port, cfg.sid,
                                             stream_type, label);
        if (!ss->is_open()) {
            cerr << "Failed to open socket for " << cfg.mcast << ":" << cfg.port << "\n";
            continue;
        }

        // addServiceToDecode triggers DecoderAdapter constructor, which pushes the sink
        if (!rx.addServiceToDecode(*ss, "", svc)) {
            cerr << "Failed to decode SID 0x" << hex << cfg.sid << dec << "\n";
            continue;
        }

        SubchannelSink* sink = dablast_pop_sink();
        if (!sink) {
            cerr << "Internal error: no sink registered for SID 0x"
                  << hex << cfg.sid << dec << "\n";
            continue;
        }

        sink->AddUntouchedStreamConsumer(ss.get());

        cerr << "Streaming SID 0x" << hex << cfg.sid << dec
              << " → " << cfg.mcast << ":" << cfg.port
              << " (stream_type=0x" << hex << (int)stream_type << dec << ")\n";

        entries.push_back({move(ss), sink, cfg.sid});
    }

    if (entries.empty()) {
        cerr << "No services to stream.\n";
        return 1;
    }

    cerr << "Streaming " << entries.size() << " service(s). Press Ctrl+C to stop.\n";

    // 6. Run until SIGINT
    signal(SIGINT, sigint_handler);
    while (g_running) {
        this_thread::sleep_for(milliseconds(200));
        // Check for signal loss
        if (ctrl.signal_lost) {
            cerr << "Signal lost.\n";
            break;
        }
    }

    // 7. Clean shutdown: remove USCs before streams are destroyed
    for (auto& e : entries) {
        e.sink->RemoveUntouchedStreamConsumer(e.stream.get());
    }

    return 0;
}

static CVirtualInput* open_rtlsdr_at_index(RadioControllerInterface& ctrl,
                                            int index)
{
#ifdef HAVE_RTLSDR
    try {
        return new CRTL_SDR(ctrl, index);
    } catch (...) {
        return nullptr;
    }
#else
    (void)index;
    return CInputFactory::GetDevice(ctrl, "rtl_sdr");
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// Frequency file parser  (one freq per line, # comments, blank lines ignored)
// ─────────────────────────────────────────────────────────────────────────────
static vector<long> parse_freq_file(const string& path)
{
    vector<long> freqs;
    ifstream f(path);
    if (!f) { cerr << "Cannot open frequency file: " << path << "\n"; return freqs; }
    string line;
    while (getline(f, line)) {
        auto pos = line.find('#');
        if (pos != string::npos) line = line.substr(0, pos);
        while (!line.empty() && isspace((unsigned char)line.back())) line.pop_back();
        if (line.empty()) continue;
        try { freqs.push_back(strtol(line.c_str(), nullptr, 10)); }
        catch (...) { cerr << "Ignoring bad frequency line: " << line << "\n"; }
    }
    return freqs;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    long   freq_hz      = 0;
    int    gain         = -1;
    int    adapter_index    = 0;
    bool   do_scan_mode = false;
    string config_file;
    string frontend     = "rtl_sdr";
    string freq_file;

    static const struct option long_opts[] = {
        {"scan",      no_argument,       nullptr, 'S'},
        {"freq-file", required_argument, nullptr, 'l'},
        {"help",      no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt, longidx;
    while ((opt = getopt_long(argc, argv, "f:a:g:F:c:l:hS", long_opts, &longidx)) != -1) {
        switch (opt) {
            case 'f': freq_hz     = strtol(optarg, nullptr, 10); break;
            case 'a': adapter_index   = atoi(optarg);  break;
            case 'g': gain        = atoi(optarg);  break;
            case 'F': frontend    = optarg;         break;
            case 'c': config_file = optarg;         break;
            case 'l': freq_file   = optarg;         break;
            case 'S': do_scan_mode = true;          break;
            case 'h': usage(argv[0]); return 0;
            default:  usage(argv[0]); return 1;
        }
    }

    // ── Validate args ────────────────────────────────────────────────────────
    if (freq_hz == 0 && freq_file.empty()) {
        cerr << "Error: -f FREQUENCY or -l FREQ_FILE is required.\n\n";
        usage(argv[0]);
        return 1;
    }
    if (!do_scan_mode && config_file.empty()) {
        cerr << "Error: specify --scan or -c CONFIG_FILE.\n\n";
        usage(argv[0]);
        return 1;
    }
    if (!freq_file.empty() && !do_scan_mode) {
        cerr << "Error: -l FREQ_FILE is only valid with --scan.\n\n";
        usage(argv[0]);
        return 1;
    }

    // ── Build frequency list ─────────────────────────────────────────────────
    vector<long> scan_freqs;
    if (!freq_file.empty()) {
        scan_freqs = parse_freq_file(freq_file);
        if (scan_freqs.empty()) {
            cerr << "No valid frequencies in: " << freq_file << "\n";
            return 1;
        }
    } else {
        scan_freqs.push_back(freq_hz);
    }

    // ── Create input device (shared across all scans) ────────────────────────
    // Use a throwaway controller just to satisfy GetDevice's constructor arg;
    // each scan gets its own fresh controller below.
    DablastController dummy_ctrl;
    unique_ptr<CVirtualInput> in(
        (frontend == "rtl_sdr")
            ? open_rtlsdr_at_index(dummy_ctrl, adapter_index)
            : CInputFactory::GetDevice(dummy_ctrl, frontend));
    if (!in) {
        cerr << "Failed to open input device '" << frontend << "'.\n"
             << "Make sure an RTL-SDR dongle is plugged in.\n";
        return 1;
    }

    if (gain < 0) {
        in->setAgc(true);
    } else {
        in->setAgc(false);
        in->setGain(gain);
    }

    // ── Streaming mode (single frequency only) ───────────────────────────────
    if (!config_file.empty()) {
        in->setFrequency((int)freq_hz);
        cerr << "dablast: tuning to " << freq_hz << " Hz"
             << " (device " << adapter_index << ")"
             << " via " << in->getDescription() << "\n";

        DablastController ctrl;
        RadioReceiverOptions rro;
        rro.decodeTII = false;
        RadioReceiver rx(ctrl, *in, rro);
        rx.restart(false);

        signal(SIGINT, sigint_handler);
        int ret = do_stream(rx, ctrl, freq_hz, config_file);
        rx.stop();
        return ret;
    }

    // ── Scan mode ─────────────────────────────────────────────────────────────
    RadioReceiverOptions rro;
    rro.decodeTII = false;

    int last_ret = 0;
    for (size_t i = 0; i < scan_freqs.size(); i++) {
        long f = scan_freqs[i];
        in->setFrequency((int)f);
        cerr << "dablast: tuning to " << f << " Hz"
             << " (device " << adapter_index << ")"
             << " via " << in->getDescription() << "\n";

        // Fresh controller so synced/ensemble state doesn't carry over
        DablastController ctrl;
        RadioReceiver rx(ctrl, *in, rro);
        rx.restart(false);

        last_ret = do_scan(rx, ctrl, f);
        rx.stop();

        // Brief pause between frequencies to let the RTL-SDR settle
        if (i + 1 < scan_freqs.size())
            this_thread::sleep_for(milliseconds(500));
    }
    return last_ret;
}
