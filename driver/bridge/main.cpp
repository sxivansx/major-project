// s2bridge: RPLIDAR S2 -> binary scan frames on stdout.
//
// The Python core server spawns this as a subprocess and reads frames from
// its stdout. The frame format is the contract in docs/api.md section 1.
//
//   s2bridge --serial <port> [--baud 1000000]      stream frames until
//                                                  SIGINT/SIGTERM, stdout
//                                                  closes (EPIPE) or stdin
//                                                  hangs up
//   s2bridge --serial <port> --info                one JSON line, exit 0
//
// Errors: one JSON line {"error":"..."} on stderr, exit 1.
// All other diagnostics go to stderr. stdout carries frames only.
//
// The connect / getDeviceInfo / getHealth / startScan / grabScanDataHq /
// ascendScanData sequence follows Slamtec's app/ultra_simple/main.cpp
// (rplidar_sdk, BSD-2). The S2 motor self-starts, so there is no
// setMotorSpeed call.

#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

namespace {

constexpr uint32_t kMagic = 0x52463253u;  // 'S2FR' when read little-endian
constexpr size_t kMaxNodes = 8192;        // S2 does ~3,200 per rotation
constexpr int kMaxGrabFailures = 5;
constexpr int kDefaultBaud = 1000000;

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "frame writer assumes a little-endian host");

volatile sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

// Minimal JSON string escaping for diagnostics and the --info line.
std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    snprintf(buf, sizeof buf, "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

// One JSON error line on stderr: {"error":"..."}
void emit_error(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void emit_error(const char* fmt, ...) {
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    fprintf(stderr, "{\"error\":\"%s\"}\n", json_escape(msg).c_str());
    fflush(stderr);
}

// Plain diagnostic line on stderr.
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void logf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    fputs("s2bridge: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    fflush(stderr);
}

void usage(FILE* to) {
    fputs("usage: s2bridge --serial <port> [--baud 1000000] [--info]\n"
          "  --serial <port>  e.g. /dev/cu.usbserial-0001\n"
          "  --baud <n>       default 1000000 (RPLIDAR S2)\n"
          "  --info           print one JSON line with device info and exit\n"
          "stream mode writes binary frames to stdout; see docs/api.md section 1\n",
          to);
}

double now_unix() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

template <typename T>
void put_le(std::vector<uint8_t>& buf, T v) {
    uint8_t tmp[sizeof(T)];
    memcpy(tmp, &v, sizeof v);
    buf.insert(buf.end(), tmp, tmp + sizeof v);
}

const char* health_name(sl_u8 status) {
    switch (status) {
        case SL_LIDAR_STATUS_OK:      return "good";
        case SL_LIDAR_STATUS_WARNING: return "warning";
        case SL_LIDAR_STATUS_ERROR:   return "error";
        default:                      return "unknown";
    }
}

std::string serial_hex(const sl_lidar_response_device_info_t& info) {
    char buf[sizeof(info.serialnum) * 2 + 1];
    for (size_t i = 0; i < sizeof(info.serialnum); ++i)
        snprintf(buf + i * 2, 3, "%02X", info.serialnum[i]);
    return buf;
}

std::string model_name(ILidarDriver* drv, const sl_lidar_response_device_info_t& info) {
    // The SDK's own table (S series starts at major id 6, so 0x71 -> "S2M1").
    // fetchAliasName=false keeps this a pure lookup, no device round trip.
    std::string name;
    if (SL_IS_OK(drv->getModelNameDescriptionString(name, false, &info)) && !name.empty())
        return name;
    char buf[32];
    snprintf(buf, sizeof buf, "model-%u", static_cast<unsigned>(info.model));
    return buf;
}

// True when stdin is a pipe/FIFO whose writer has gone away. A tty, a
// regular file, /dev/null or a live pipe never trigger this.
bool stdin_hung_up() {
    if (isatty(STDIN_FILENO)) return false;
    struct pollfd p;
    p.fd = STDIN_FILENO;
    p.events = POLLIN;
    p.revents = 0;
    if (poll(&p, 1, 0) <= 0) return false;
    return (p.revents & POLLHUP) != 0;
}

// Owns the SDK objects so every exit path tears them down in order:
// stop scan (caller), disconnect, delete driver, delete channel. The SDK
// closes the channel on disconnect but never frees it.
struct Lidar {
    ILidarDriver* drv = nullptr;
    IChannel* channel = nullptr;

    ~Lidar() {
        if (drv) {
            if (drv->isConnected()) drv->disconnect();
            delete drv;
        }
        delete channel;
    }
};

// Build one frame from a full rotation and write it in a single fwrite.
// Returns false when the write fails (EPIPE when the parent is gone).
bool write_frame(FILE* out, const sl_lidar_response_measurement_node_hq_t* nodes,
                 size_t count, double t_unix, std::vector<uint8_t>& buf) {
    buf.clear();
    put_le(buf, kMagic);
    put_le(buf, static_cast<uint32_t>(0));  // n, patched below
    put_le(buf, t_unix);

    uint32_t n = 0;
    for (size_t i = 0; i < count; ++i) {
        const sl_lidar_response_measurement_node_hq_t& nd = nodes[i];
        if (nd.dist_mm_q2 == 0) continue;  // no return at this angle
        const float angle_deg = nd.angle_z_q14 * 90.f / 16384.f;
        const float range_m = nd.dist_mm_q2 / 4.0f / 1000.f;
        const uint8_t quality =
            static_cast<uint8_t>(nd.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        put_le(buf, angle_deg);
        put_le(buf, range_m);
        buf.push_back(quality);
        ++n;
    }
    memcpy(&buf[4], &n, sizeof n);

    if (fwrite(buf.data(), 1, buf.size(), out) != buf.size()) return false;
    if (fflush(out) != 0) return false;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const char* port = nullptr;
    int baud = kDefaultBaud;
    bool info_only = false;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (strcmp(a, "--serial") == 0 && i + 1 < argc) {
            port = argv[++i];
        } else if (strcmp(a, "--baud") == 0 && i + 1 < argc) {
            char* end = nullptr;
            long v = strtol(argv[++i], &end, 10);
            if (!end || *end != '\0' || v <= 0 || v > 10000000) {
                emit_error("invalid --baud value: %s", argv[i]);
                return 1;
            }
            baud = static_cast<int>(v);
        } else if (strcmp(a, "--info") == 0) {
            info_only = true;
        } else if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(stderr);
            return 0;
        } else {
            emit_error("unknown or incomplete argument: %s", a);
            usage(stderr);
            return 1;
        }
    }
    if (!port) {
        emit_error("--serial <port> is required");
        usage(stderr);
        return 1;
    }

    // Keep the real stdout private for frames. The SDK printf()s to stdout
    // (e.g. "Setting serial port baudrate..." in arch/macOS/net_serial.cpp),
    // which would corrupt the stream, so fd 1 is pointed at stderr for the
    // rest of the process and frames go to a dup of the original fd.
    int frame_fd = dup(STDOUT_FILENO);
    if (frame_fd < 0 || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
        emit_error("cannot set up stdout: %s", strerror(errno));
        return 1;
    }
    setvbuf(stdout, nullptr, _IONBF, 0);
    FILE* out = fdopen(frame_fd, "wb");
    if (!out) {
        emit_error("cannot open frame sink: %s", strerror(errno));
        return 1;
    }
    setvbuf(out, nullptr, _IONBF, 0);  // one write() per frame

    // A dead parent shows up as EPIPE from fwrite, never as a signal.
    signal(SIGPIPE, SIG_IGN);
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;  // the SDK's threads never see EINTR
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // Probe the port first. The SDK's connect() reports success even when the
    // open fails (openChannelAndBind shadows its own result variable), so this
    // is the only place a precise reason (missing, busy, not a tty) is known.
    {
        int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            emit_error("cannot open serial port %s: %s%s", port, strerror(errno),
                       errno == EBUSY ? " (in use by another process?)" : "");
            return 1;
        }
        const bool tty = isatty(fd);
        close(fd);
        if (!tty) {
            emit_error("%s is not a serial device", port);
            return 1;
        }
    }

    Lidar lidar;

    Result<IChannel*> ch = createSerialPortChannel(port, baud);
    if (!ch || !*ch) {
        emit_error("cannot create serial channel for %s", port);
        return 1;
    }
    lidar.channel = *ch;

    Result<ILidarDriver*> drv = createLidarDriver();
    if (!drv || !*drv) {
        emit_error("cannot create lidar driver (out of memory?)");
        return 1;
    }
    lidar.drv = *drv;

    sl_result res = lidar.drv->connect(lidar.channel);
    if (SL_IS_FAIL(res)) {
        emit_error("cannot open serial port %s at %d baud (0x%x)", port, baud,
                   static_cast<unsigned>(res));
        return 1;
    }

    sl_lidar_response_device_info_t devinfo;
    memset(&devinfo, 0, sizeof devinfo);
    res = lidar.drv->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(res)) {
        if (res == SL_RESULT_OPERATION_NOT_SUPPORT)
            // The SDK's open() failed (termios or IOSSIOSPEED); its own message
            // is on stderr above.
            emit_error("SDK could not set up serial port %s at %d baud", port, baud);
        else if (res == SL_RESULT_OPERATION_TIMEOUT)
            emit_error("no response from %s at %d baud: no lidar on this port or wrong baud",
                       port, baud);
        else
            emit_error("no device info from %s at %d baud (0x%x)", port, baud,
                       static_cast<unsigned>(res));
        return 1;
    }

    sl_lidar_response_device_health_t health;
    memset(&health, 0, sizeof health);
    res = lidar.drv->getHealth(health);
    if (SL_IS_FAIL(res)) {
        emit_error("cannot read lidar health (0x%x)", static_cast<unsigned>(res));
        return 1;
    }

    const std::string model = model_name(lidar.drv, devinfo);
    const std::string serial = serial_hex(devinfo);
    char firmware[16];
    snprintf(firmware, sizeof firmware, "%d.%02d", devinfo.firmware_version >> 8,
             devinfo.firmware_version & 0xFF);

    if (info_only) {
        fprintf(out,
                "{\"model\":\"%s\",\"model_id\":%u,\"firmware\":\"%s\",\"hardware\":%u,"
                "\"serial\":\"%s\",\"health\":\"%s\",\"health_code\":%u}\n",
                json_escape(model).c_str(), static_cast<unsigned>(devinfo.model), firmware,
                static_cast<unsigned>(devinfo.hardware_version), serial.c_str(),
                health_name(health.status), static_cast<unsigned>(health.status));
        fflush(out);
        if (health.error_code)
            logf("health error code 0x%04x", static_cast<unsigned>(health.error_code));
        return 0;
    }

    logf("connected %s @ %d: %s (id %u) fw %s hw %u serial %s health %s", port, baud,
         model.c_str(), static_cast<unsigned>(devinfo.model), firmware,
         static_cast<unsigned>(devinfo.hardware_version), serial.c_str(),
         health_name(health.status));
    if (health.status == SL_LIDAR_STATUS_ERROR) {
        emit_error("lidar health error (code 0x%04x); power-cycle the device",
                   static_cast<unsigned>(health.error_code));
        return 1;
    }
    if (health.status == SL_LIDAR_STATUS_WARNING)
        logf("health warning (code 0x%04x), continuing", static_cast<unsigned>(health.error_code));

    LidarScanMode mode;
    memset(&mode, 0, sizeof mode);
    res = lidar.drv->startScan(false, true, 0, &mode);
    if (SL_IS_FAIL(res)) {
        emit_error("startScan failed (0x%x)", static_cast<unsigned>(res));
        return 1;
    }
    logf("scanning: mode %u \"%s\" %.2f us/sample max %.1f m", static_cast<unsigned>(mode.id),
         mode.scan_mode, static_cast<double>(mode.us_per_sample),
         static_cast<double>(mode.max_distance));

    static sl_lidar_response_measurement_node_hq_t nodes[kMaxNodes];
    std::vector<uint8_t> frame;
    frame.reserve(16 + kMaxNodes * 9);

    int exit_code = 0;
    int grab_failures = 0;
    unsigned long frames = 0;

    while (!g_stop) {
        if (stdin_hung_up()) {
            logf("stdin closed, stopping");
            break;
        }

        size_t count = kMaxNodes;
        res = lidar.drv->grabScanDataHq(nodes, count);
        if (g_stop) break;
        if (SL_IS_FAIL(res)) {
            ++grab_failures;
            logf("grabScanDataHq failed (0x%x), %d/%d", static_cast<unsigned>(res),
                 grab_failures, kMaxGrabFailures);
            if (grab_failures > kMaxGrabFailures) {
                emit_error("grabScanDataHq failed %d times in a row (last 0x%x); giving up",
                           grab_failures, static_cast<unsigned>(res));
                exit_code = 1;
                break;
            }
            continue;
        }
        grab_failures = 0;

        const double t_unix = now_unix();
        lidar.drv->ascendScanData(nodes, count);

        if (!write_frame(out, nodes, count, t_unix, frame)) {
            if (errno == EPIPE)
                logf("stdout closed (parent gone), stopping");
            else
                logf("stdout write failed: %s", strerror(errno));
            break;
        }
        if (++frames == 1)
            logf("streaming: first frame has %zu nodes", count);
    }

    if (g_stop) logf("signal received, stopping");
    logf("stopping after %lu frames", frames);
    lidar.drv->stop();
    // ~Lidar disconnects, deletes the driver, then the channel.
    return exit_code;
}
