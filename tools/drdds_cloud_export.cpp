// ============================================================================
// drdds_cloud_export.cpp — NEW-09 minimal vendor-SDK point cloud exporter
// Target: GOS host (ssh 104), RoboSense M20 DrDDS (libdrdds 1.1.7 / FastDDS 2.14)
//
// Purpose:
//   Standalone vendor-DrDDS process. Subscribes ONLY /LIDAR/POINTS
//   (sensor_msgs::msg::PointCloud2, local SHM participant use_shm=true — the
//   proven NEW-07 receive path) and re-publishes the frames verbatim on
//   /new_chase/points (DrDDSPublisher, use_shm=false = plain UDP, multi-NIC).
//   Domain 0, prefix "rt". This is NOT a bridge implementation inside ROS:
//   Foxy ROS only consumes /new_chase/points over UDP with the NEW-02
//   diagnostic_udp_only.xml (separate process, owned by tester).
//
// Hard scope limits (enforced in code, not by convention):
//   - The ONLY subscribed topic is the fixed constant kInTopic below.
//   - The ONLY published topic is the fixed constant kOutTopic below.
//   - No CLI parameter can change either topic. No NavCmd / no motion / no
//     heartbeat / no mode / no control message of any kind is ever
//     constructed or published (no other DrDDSPublisher/DrDDSChannel exists
//     in this translation unit).
//   - stamp / frame_id / fields / data bytes are forwarded verbatim; this
//     code never writes to the message content.
//
// Static verification backing the callback->Write path (NEW-09, libdrdds
// 1.1.7, objdump evidence; see evidence/NEW-09-export.md):
//   - DrDDSPublisher<PC2>::Write(const Type*) @0xe6306c takes the caller's
//     pointer only to immediately deep-copy the full sample into its own
//     async queue via std::make_shared<sensor_msgs::msg::PointCloud2>(
//     sensor_msgs::msg::PointCloud2 const&) @0xe63118, under its internal
//     mutex (this+0x80), then notify_one (this+0xb0). When the queue is
//     full (>= max_queue_size_ at this+0xE0) it pops the oldest first.
//     => after Write() returns, the callback's data pointer is no longer
//        referenced: no dangling-pointer lifetime risk in the vendor path.
//   - Worker thread: ctor @0xe626fc calls StartWorkerThread() @plt e62dc8
//     internally; the worker lambda delegates to WorkerThread() @0xe6390c
//     which drains the queue and calls
//     eprosima::fastdds::dds::DataWriter::write(void*) @0xe63a94.
//     => we must NOT call StartWorkerThread() ourselves (double thread).
//   - ~DrDDSPublisher @0xe62ef4: atomic stop store + notify_all +
//     joinable/join + delete_datawriter + delete_publisher + ~DrDDSMessage
//     — graceful vendor-side teardown confirmed.
//   - SetQueueSize(unsigned long) @0xe63810 stores the limit under the same
//     queue mutex; Write pops-then-pushes when size >= limit (latest-wins).
//
// NEW-09 fix round 1 — publisher participant routability (evidence-backed):
//   First run: SDK input fine (rx249 write_ok249 write_err0), but the ROS
//   UDP-only reader discovered /new_chase/points count=0 publishers=[] —
//   the SDK output writer was not discoverable over the routable interface.
//   Root cause verified statically in DrDDSManager::Init(int, std::string)
//   @0xdcb188..0xdcb94c (libdrdds 1.1.7):
//     * Init creates TWO participants. The second ("multi", UDP) one gets a
//       UDPv4 descriptor whose interfaceWhiteList (descriptor+0x90) is
//       seeded with a hardcoded C-string literal first entry
//       (@0xdcb600..0xdcb62c, literal page 0x15ee000+0x4e8), and then —
//       only when network_name is non-empty — the segments of network_name
//       SPLIT BY '/' (@0xdcb664 string::find(char 0x2f='/'),
//       @0xdcb6b8/@0xdcb704 substr + @0xdcb6c8/@0xdcb714 push_back).
//     => network_name is a '/'-separated IP whitelist, NOT an interface
//        name and NOT comma-separated. With Init(0) (empty network_name)
//        the multi participant UDP whitelist is loopback-only — consistent
//        with the ROS reader never seeing the writer.
//     => minimal fix: Init(0, "10.21.31.104") — multi participant UDP
//        whitelist becomes {default, 10.21.31.104}, i.e. routable.
//     The FIRST (local) participant (SHM ~50MiB + UDP loopback, used by the
//     use_shm=true input side) is built in the earlier block and does NOT
//     consume network_name — the NEW-07-proven input path is untouched.
//     If the rerun still shows the ROS reader discovering nothing, the
//     documented fallback applies (explicit native Fast-DDS participant
//     for the outbound writer — evidence/NEW-09-export.md §7).
//
// Resource side effects (declared, not hidden):
//   - use_shm=true on the input participant makes libdrdds create a ~50 MiB
//     SHM segment (vendor-internal size, unavoidable; disclosed). This
//     process NEVER unlinks/removes/cleans any SHM file or lock; nothing is
//     cleaned automatically on exit or signal (no unlink/shm_unlink/system/
//     popen anywhere). SHM before/after snapshots are external (main agent).
//   - Output participant use_shm=false creates no SHM segment of its own.
//   - Root-only by design (vendor SHM segments are root-owned; NEW-07
//     acceptance proved user cannot attach). Not for Web/algorithm users;
//     they consume /new_chase/points via ROS instead.
//
// Environment contract (checked at startup, hard-failed otherwise):
//   - Must run as root (geteuid()==0), else exit 2.
//   - FASTRTPS_DEFAULT_PROFILES_FILE / FASTDDS_DEFAULT_PROFILES_FILE /
//     FASTDDS_ENVIRONMENT_FILE must be UNSET, else exit 2 (prevents
//     accidentally re-creating the vendor 500 MB SHM profile or any
//     process-level XML override).
//   - LD_LIBRARY_PATH containing "/opt/ros" is rejected, else exit 2
//     (this process must never load the Foxy ROS stack).
//
// Lifecycle:
//   - --seconds N, bounded, default 15, clamp 1..120. --continuous runs
//     until SIGINT/SIGTERM (banner states Ctrl+C; exports point cloud only;
//     not a daemon; no auto-restart).
//   - SIGINT/SIGTERM set a flag only (async-safe); the main loop exits,
//     callbacks are fenced under the callback mutex, then destruction
//     order is: subscriber first, then publisher, then
//     DrDDSManager::Delete(). Only internal DDS library resources are
//     released; no files are touched.
//
// Output: banner lines on stderr; one single-line JSON summary on stdout.
// Exit codes: 0 = graceful bounded/continuous run ended with rx>0 and no
//             enqueue errors (summary printed; 0 does NOT claim a verified
//             successful end-to-end flow — that is judged externally),
//             1 = graceful run ended but rx==0 or write enqueue errors,
//             2 = argument/environment/privilege error,
//             3 = runtime/SDK exception (cleanup attempted, nonzero).
// ============================================================================

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <unistd.h>

#include <drdds/core/drdds_core.h>

namespace {

// ------------------------------------------------------------------ constants
constexpr char    kInTopic[]   = "/LIDAR/POINTS";
constexpr char    kOutTopic[]  = "/new_chase/points";
constexpr char    kLocalIps[]  = "10.21.31.104";  // '/'-separated IP list for DrDDSManager::Init
constexpr int     kDomain      = 0;
constexpr char    kPrefix[]    = "rt";
constexpr bool    kInUseShm    = true;   // proven receive path (NEW-07, root)
constexpr bool    kUseShmOut   = false;  // plain UDP, multi-NIC
constexpr int     kMinSeconds  = 1;
constexpr int     kMaxSeconds  = 120;
constexpr int     kDefaultSec  = 15;
constexpr size_t  kQueueSize   = 1;      // latest-wins, no backlog

// ------------------------------------------------------------- process state
volatile sig_atomic_t g_sig = 0;            // async-signal-safe flag only
std::atomic<bool>     g_stopping{false};
int                   g_exit_code = 0;      // from JSON summary block

std::mutex            g_cb_mutex;           // fences callback vs shutdown
DrDDSPublisher<sensor_msgs::msg::PointCloud2PubSubType>* g_pub = nullptr;

// statistics (counters atomic; stamp/width snapshots under g_stats_mutex)
std::atomic<uint64_t> g_rx{0}, g_write_ok{0}, g_write_err{0};
std::atomic<int>      g_pub_matched_max{0};   // polled at 20 Hz in the run loop
std::mutex            g_stats_mutex;
double                g_age_last = -1.0, g_age_max = -1.0;
uint32_t              g_last_width = 0, g_last_point_step = 0, g_last_height = 0;
size_t                g_last_fields = 0, g_last_data_bytes = 0;
std::string           g_last_frame_id;
std::chrono::steady_clock::time_point g_first_rx{}, g_last_rx{};
bool                  g_first_rx_set = false;

// ------------------------------------------------------------------- helpers
double steady_now_s()
{
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

double wall_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string json_escape(const std::string& in)
{
  std::ostringstream os;
  char buf[8];
  for (size_t i = 0; i < in.size(); ++i) {
    const unsigned char c = static_cast<unsigned char>(in[i]);
    switch (c) {
      case '"':  os << "\\\""; break;
      case '\\': os << "\\\\"; break;
      case '\b': os << "\\b";  break;
      case '\f': os << "\\f";  break;
      case '\n': os << "\\n";  break;
      case '\r': os << "\\r";  break;
      case '\t': os << "\\t";  break;
      default:
        if (c < 0x20) { std::snprintf(buf, sizeof(buf), "\\u%04x", c); os << buf; }
        else          { os << in[i]; }
    }
  }
  return os.str();
}

extern "C" void on_signal(int)
{
  g_sig = 1;   // flag only; all logic happens in the main thread
}

// ------------------------------------------------------------------- callback
// Runs on the vendor subscriber callback thread. Write() deep-copies the
// sample into the publisher's async queue (verified @0xe63118), so passing
// the callback pointer is safe and no manual copy is needed.
void on_cloud(const sensor_msgs::msg::PointCloud2* msg)
{
  if (msg == nullptr) return;
  std::lock_guard<std::mutex> lk(g_cb_mutex);
  if (g_stopping.load(std::memory_order_acquire)) return;

  g_rx.fetch_add(1, std::memory_order_relaxed);

  // stats snapshot (verbatim fields; nothing is modified)
  {
    std::lock_guard<std::mutex> sk(g_stats_mutex);
    const builtin_interfaces::msg::Time& st = msg->header().stamp();
    const double age_s =
        (wall_now_ns() -
         (static_cast<double>(st.sec()) * 1e9 +
          static_cast<double>(st.nanosec()))) / 1e9;
    g_age_last    = age_s;
    if (age_s > g_age_max) g_age_max = age_s;
    g_last_width      = msg->width();
    g_last_height     = msg->height();
    g_last_point_step = msg->point_step();
    g_last_fields     = msg->fields().size();
    g_last_data_bytes = msg->data().size();
    g_last_frame_id   = msg->header().frame_id();

    const auto now = std::chrono::steady_clock::now();
    if (!g_first_rx_set) { g_first_rx = now; g_first_rx_set = true; }
    g_last_rx = now;
  }

  if (g_pub != nullptr) {
    if (g_pub->Write(msg)) g_write_ok.fetch_add(1, std::memory_order_relaxed);
    else                   g_write_err.fetch_add(1, std::memory_order_relaxed);
  } else {
    g_write_err.fetch_add(1, std::memory_order_relaxed);
  }
}

// ----------------------------------------------------------------- env checks
bool env_forbidden(const char* name)
{
  return ::getenv(name) != nullptr;
}

bool ldpath_has_ros()
{
  const char* lp = ::getenv("LD_LIBRARY_PATH");
  if (lp == nullptr) return false;
  return std::strstr(lp, "/opt/ros") != nullptr;
}

}  // namespace

// ======================================================================= main
int main(int argc, char** argv)
{
  // ---------------- arguments
  int  seconds    = kDefaultSec;
  bool continuous = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--seconds" && i + 1 < argc) {
      char* end = nullptr;
      const long v = std::strtol(argv[++i], &end, 10);
      if (end == argv[i + 1] || *end != '\0' || v < kMinSeconds || v > kMaxSeconds) {
        std::cerr << "[drdds_cloud_export] --seconds must be an integer in ["
                  << kMinSeconds << "," << kMaxSeconds << "]\n";
        return 2;
      }
      seconds = static_cast<int>(v);
    } else if (a == "--seconds") {
      std::cerr << "[drdds_cloud_export] --seconds requires a value\n";
      return 2;
    } else if (a == "--continuous") {
      continuous = true;
    } else {
      std::cerr << "[drdds_cloud_export] unknown argument: " << a << "\n";
      return 2;
    }
  }

  // ---------------- environment / privilege contract (hard errors)
  if (::geteuid() != 0) {
    std::cerr << "[drdds_cloud_export] must run as root (geteuid()==0); "
                 "user attach to the vendor SHM cloud is not possible (NEW-07)\n";
    return 2;
  }
  if (env_forbidden("FASTRTPS_DEFAULT_PROFILES_FILE") ||
      env_forbidden("FASTDDS_DEFAULT_PROFILES_FILE") ||
      env_forbidden("FASTDDS_ENVIRONMENT_FILE")) {
    std::cerr << "[drdds_cloud_export] refusing to start: FASTRTPS/FASTDDS "
                 "profile env vars are set; run with env -i and NO profile "
                 "variables (avoid vendor 500MB SHM / XML overrides)\n";
    return 2;
  }
  if (ldpath_has_ros()) {
    std::cerr << "[drdds_cloud_export] refusing to start: LD_LIBRARY_PATH "
                 "contains /opt/ros (Foxy stack must not be loaded)\n";
    return 2;
  }

  // ---------------- signal disposition (flags only; set before Init)
  ::signal(SIGINT,  on_signal);
  ::signal(SIGTERM, on_signal);

  // ---------------- banner (stderr; stdout stays clean for the JSON)
  std::cerr << "[drdds_cloud_export] pid=" << ::getpid()
            << " mode=" << (continuous ? "continuous" : "bounded")
            << " seconds=" << (continuous ? 0 : seconds) << "\n";
  if (continuous) {
    std::cerr << "[drdds_cloud_export] continuous mode: press Ctrl+C to stop\n";
  }
  std::cerr << "[drdds_cloud_export] subscriber: topic=" << kInTopic
            << " use_shm=true (DrDDSManager local participant: SHM ~50MiB + "
               "UDP loopback; unchanged from the NEW-07-proven path)\n";
  std::cerr << "[drdds_cloud_export] publisher: topic=rt" << kOutTopic
            << " (ROS name " << kOutTopic << ")"
            << " type=sensor_msgs::msg::dds_::PointCloud2_"
               " (vendor PointCloud2PubSubType)"
            << " use_shm=false (DrDDSManager multi participant, UDP)\n";
  std::cerr << "[drdds_cloud_export] manager: Init(0, \"" << kLocalIps
            << "\") -> multi participant UDP whitelist = vendor-hardcoded "
               "loopback entry + '/'-split arg (libdrdds 0xdcb600..0xdcb718); "
               "write_ok counts vendor queue-enqueue success, NOT wire "
               "delivery\n";
  std::cerr << "[drdds_cloud_export] exports ONLY the point cloud "
            << kInTopic << " -> " << kOutTopic
            << ". NO control/motion/heartbeat topic is subscribed or "
               "published. NOT a daemon.\n";
  std::cerr << "[drdds_cloud_export] note: libdrdds internal local SHM "
               "(~50MiB, use_shm=true input side) is unavoidable and is "
               "disclosed; this process never cleans it.\n";

  const double t0 = steady_now_s();

  try {
    // ---- SDK init: input side identical call family to the NEW-07-proven
    // probe (domain 0); network_name added for the OUTPUT participant only:
    // '/'-separated IP whitelist, verified @0xdcb664/dcb6b8/dcb714.
    DrDDSManager::Init(0, kLocalIps);

    // ---------------- publisher first (callback needs it), then subscriber;
    // scope guarantees destruction order: subscriber, then publisher, then
    // Manager::Delete() — per NEW-07 release-order review.
    {
      DrDDSPublisher<sensor_msgs::msg::PointCloud2PubSubType>
          pub(kOutTopic, kDomain, kPrefix, kUseShmOut);
      pub.SetQueueSize(kQueueSize);   // latest-wins; verified @0xe63810
      g_pub = &pub;

      {
        DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType>
            sub(&on_cloud, kInTopic, kDomain, kPrefix, kInUseShm);

        const int sub_matched_init = sub.GetMatchedCount();

        // ---------------- run loop (bounded or continuous)
        // 50 ms period = 20 Hz matched-count polling (main agent requested
        // pub_max_matched because a single end snapshot is unreliable when
        // the ROS probe may exit earlier).
        bool timed_out = false;
        const double deadline = t0 + static_cast<double>(seconds);
        while (!g_sig) {
          if (!continuous && steady_now_s() >= deadline) { timed_out = true; break; }
          const int pm = pub.GetMatchedCount();
          int prev = g_pub_matched_max.load(std::memory_order_relaxed);
          while (pm > prev &&
                 !g_pub_matched_max.compare_exchange_weak(
                     prev, pm, std::memory_order_relaxed)) {}
          struct timespec ts{0, 50 * 1000 * 1000};  // 50 ms
          ::nanosleep(&ts, nullptr);
        }
        const bool interrupted = g_sig != 0;

        // ---------------- graceful shutdown (order: sub, pub, Manager)
        {
          std::lock_guard<std::mutex> lk(g_cb_mutex);
          g_stopping.store(true, std::memory_order_release);
        }
        // subscriber destroyed at scope exit (vendor dtor stops callback
        // thread; NEW-07 verified subscriber teardown)
        const int sub_matched_last = sub.GetMatchedCount();

        // ensure no callback is mid-flight before touching the publisher
        { std::lock_guard<std::mutex> lk(g_cb_mutex); }

        const int pub_matched_last = pub.GetMatchedCount();
        // publisher destroyed at scope exit (vendor dtor: stop + join +
        // delete_datawriter + delete_publisher, verified @0xe62ef4)
        g_pub = nullptr;

        // ---------------- JSON summary (stdout, single line)
        {
          std::lock_guard<std::mutex> sk(g_stats_mutex);
          const double ran = steady_now_s() - t0;
          // (count-1)/elapsed over first->last frame, 0 for <2 frames
          double dt = 0.0;
          if (g_first_rx_set && g_last_rx > g_first_rx) {
            dt = std::chrono::duration<double>(g_last_rx - g_first_rx).count();
          }
          const double rx_hz =
              (g_rx.load() >= 2 && dt > 0.0)
                  ? static_cast<double>(g_rx.load() - 1) / dt : 0.0;
          const double write_hz =
              (g_write_ok.load() >= 2 && dt > 0.0)
                  ? static_cast<double>(g_write_ok.load() - 1) / dt
                  : 0.0;
          // exit contract: nonzero when nothing received or enqueue errors
          const int exit_code =
              (g_rx.load() == 0 || g_write_err.load() > 0) ? 1 : 0;
          std::ostringstream os;
          os << "{\"tool\":\"drdds_cloud_export\",\"pid\":" << ::getpid()
             << ",\"euid\":" << ::geteuid()
             << ",\"mode\":\"" << (continuous ? "continuous" : "bounded") << "\""
             << ",\"requested_seconds\":" << (continuous ? 0 : seconds)
             << ",\"ran_seconds\":" << ran
             << ",\"interrupted\":" << (interrupted ? "true" : "false")
             << ",\"timed_out\":" << (timed_out ? "true" : "false")
             << ",\"input_topic\":\"" << kInTopic << "\""
             << ",\"input_use_shm\":" << (kInUseShm ? "true" : "false")
             << ",\"output_topic\":\"" << kOutTopic << "\""
             << ",\"output_use_shm\":" << (kUseShmOut ? "true" : "false")
             << ",\"domain\":" << kDomain
             << ",\"init_network_arg\":\"" << kLocalIps << "\""
             << ",\"queue_size\":" << kQueueSize
             << ",\"rx\":" << g_rx.load()
             << ",\"write_enqueued_ok\":" << g_write_ok.load()
             << ",\"write_enqueued_err\":" << g_write_err.load()
             << ",\"write_note\":\"vendor enqueue success, not wire delivery\""
             << ",\"sub_matched_init\":" << sub_matched_init
             << ",\"sub_matched_last\":" << sub_matched_last
             << ",\"pub_matched_last\":" << pub_matched_last
             << ",\"pub_matched_max\":" << g_pub_matched_max.load()
             << ",\"rx_hz\":" << rx_hz
             << ",\"write_hz\":" << write_hz
             << ",\"stamp_age_last_s\":" << g_age_last
             << ",\"stamp_age_max_s\":" << g_age_max
             << ",\"last_frame_id\":\"" << json_escape(g_last_frame_id) << "\""
             << ",\"last_width\":" << g_last_width
             << ",\"last_height\":" << g_last_height
             << ",\"last_point_step\":" << g_last_point_step
             << ",\"last_fields\":" << g_last_fields
             << ",\"last_data_bytes\":" << g_last_data_bytes
             << ",\"exit_code\":" << exit_code
             << ",\"note\":\"SDK input path proven (NEW-07); export "
                "end-to-end NOT yet verified; write_enqueued_ok is vendor "
                "queue-enqueue success only\""
             << "}";
          std::cout << os.str() << std::endl;
          g_exit_code = exit_code;
        }
      }  // subscriber destroyed here (before publisher)

      // drain-fence: no callback can be running once the subscriber is gone
      { std::lock_guard<std::mutex> lk(g_cb_mutex); }

    }  // publisher destroyed here (vendor graceful teardown)

    DrDDSManager::Delete();
    return g_exit_code;  // 0 = graceful; 1 = ran but rx==0 or enqueue errors;
                         // 0/1 do NOT claim a verified end-to-end flow

  } catch (const std::exception& e) {
    std::cerr << "[drdds_cloud_export] exception: " << e.what() << "\n";
  } catch (...) {
    std::cerr << "[drdds_cloud_export] unknown exception\n";
  }
  DrDDSManager::Delete();   // best-effort paired cleanup on failure path
  return 3;
}
