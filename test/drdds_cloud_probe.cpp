// ============================================================================
// drdds_cloud_probe.cpp — NEW-07 minimal subscription-only acceptance probe
// Target: GOS host (ssh 104), RoboSense M20 DrDDS (libdrdds 1.1.7 / FastDDS 2.14)
//
// Purpose:
//   Reuse the vendor DrDDS SDK in its smallest form to receive ONLY
//   /LIDAR/POINTS (sensor_msgs::msg::PointCloud2) for a bounded number of
//   seconds and report frame statistics as JSON on stdout.
//   This is NOT a production bridge: no publishing, no forwarding, no
//   velocity/motion/heartbeat output, no control of the robot.
//
// Design facts verified statically (NEW-07 static review, see evidence):
//   - DrDDSSubscriber<T> ctor: (CallBack, topic_name, domain_id,
//     topic_prefix, use_shm=false). Header-declared members are ONLY
//     DomainParticipant/Subscriber/DataReader (+callback thread): the
//     subscriber constructor does NOT create a Publisher/DataWriter
//     (DrDDSChannel does; we deliberately do not use DrDDSChannel).
//   - Vendor rslidar publishes the point cloud through a
//     DrDDSChannel<...PointCloud2PubSubType> constructed with use_shm=true,
//     domain 0, prefix "rt" (disassembly evidence, NEW-04 §11 / NEW-07 §3);
//     vendor drddsctl echo also constructs its point cloud channel with
//     use_shm=true (disassembly evidence, NEW-07 §3). Hence use_shm=true here.
//   - DrDDSManager::Init(int, const std::string network_name = "") builds the
//     default vendor participant path; we call Init(0) only (no network_name
//     guesswork), then DrDDSManager::Delete() AFTER the subscriber object is
//     fully destroyed (RAII scope), per the release-order review of
//     drdds_core.h / drdds_manager.h.
//
// Resource side effects (declared, not hidden):
//   - use_shm=true makes libdrdds create FastDDS SHM segments in /dev/shm
//     (one ~50 MB data segment plus listeners is the observed vendor size).
//     This probe is therefore NOT absolutely free of resource side effects.
//     This code NEVER unlinks/removes/cleans any SHM file; nothing is
//     cleaned automatically on exit or on signal (no shm_unlink/remove/
//     unlink/system/popen anywhere). SHM before/after snapshots and
//     statvfs are done externally by the main agent via shell.
//   - Normal-user run only; no automatic root retry. Root permission
//     comparison is exclusively a main-agent action.
//
// Output: key=value banner line, then a single-line JSON report.
// Exit codes: 0 = at least one valid fresh frame at exit;
//             1 = ran to completion (or interrupted) without fresh frames;
//             2 = argument/environment/exception error.
// ============================================================================

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <drdds/core/drdds_core.h>

#include <unistd.h>   // getuid / geteuid
#include <sys/resource.h>

namespace {

// ---------------------------------------------------------------- constants
constexpr char     kTopic[]        = "/LIDAR/POINTS";
constexpr int      kDomain         = 0;
constexpr char     kPrefix[]       = "rt";
constexpr bool     kUseShm         = true;   // matches vendor point cloud writer
constexpr double   kFreshMaxAge    = 0.5;    // source stamp age upper bound  [s]
constexpr double   kFreshMaxFuture = 0.1;    // allowed future stamp margin   [s]
constexpr double   kRecvFreshMax   = 0.5;    // receive recency upper bound   [s]
constexpr int      kMinSeconds     = 1;
constexpr int      kMaxSeconds     = 20;
constexpr int      kDefaultSeconds = 15;

volatile sig_atomic_t g_quit = 0;

std::chrono::steady_clock::time_point g_start_steady;

// ------------------------------------------------------------------- helpers
double steady_since_start()
{
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now() - g_start_steady).count();
}

double wall_now()
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(
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
        if (c < 0x20) {
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          os << buf;
        } else {
          os << in[i];
        }
    }
  }
  return os.str();
}

// num_as_json: finite -> text; NaN/inf -> null (no raw NaN inside JSON)
std::string num_as_json(double v, int precision)
{
  std::ostringstream os;
  os << std::fixed;
  os.precision(precision);
  if (!std::isfinite(v)) {
    return "null";
  }
  os << v;
  return os.str();
}

std::string num_as_json(double v) { return num_as_json(v, 6); }

// ---------------------------------------------------------------- statistics
struct FieldInfo
{
  std::string name;
  uint32_t    offset   = 0;
  uint8_t     datatype = 0;
};

struct Stats
{
  std::mutex m;

  uint64_t frames       = 0;
  uint64_t valid_frames = 0;
  bool     latest_valid = false;

  std::chrono::steady_clock::time_point first_rx;
  std::chrono::steady_clock::time_point last_rx;
  std::chrono::steady_clock::time_point last_valid_rx;
  bool has_last_valid_rx = false;

  // last received frame (raw values, not rewritten)
  uint32_t width       = 0;
  uint32_t height      = 0;
  uint32_t point_step  = 0;
  uint32_t row_step    = 0;
  uint64_t data_bytes  = 0;
  bool     is_bigendian = false;
  bool     is_dense     = false;

  // last VALID frame
  std::string last_valid_frame_id;
  int32_t     last_valid_sec     = 0;
  uint32_t    last_valid_nanosec = 0;

  // field layout of the FIRST received frame
  int                 field_count = 0;
  bool                xyz_present = false;
  std::vector<FieldInfo> first_fields;

  // source stamp age (wall_now - stamp) over VALID frames, [s]
  double age_min  = 0.0;
  double age_max  = 0.0;
  double age_last = 0.0;
  bool   has_age  = false;

  std::string last_invalid_reason;
};

Stats g_stats;

bool has_xyz_fields(const std::vector<sensor_msgs::msg::PointField>& fields)
{
  bool x = false, y = false, z = false;
  for (size_t i = 0; i < fields.size(); ++i) {
    const std::string& n = fields[i].name();
    if (n == "x" || n == "X") x = true;
    if (n == "y" || n == "Y") y = true;
    if (n == "z" || n == "Z") z = true;
  }
  return x && y && z;
}

// Frame validity (statistics-level checks only; stamps are never rewritten).
bool frame_is_valid(const sensor_msgs::msg::PointCloud2& m, std::string& reason)
{
  if (m.width() == 0) { reason = "width_zero"; return false; }
  if (m.point_step() == 0) { reason = "point_step_zero"; return false; }

  const size_t need_points =
      static_cast<size_t>(m.height()) * m.width() * m.point_step();
  const size_t need_rows =
      m.row_step() == 0
          ? need_points
          : static_cast<size_t>(m.row_step()) * m.height();
  const size_t need = need_points > need_rows ? need_points : need_rows;

  if (m.data().size() < need) { reason = "data_too_short"; return false; }
  if (m.row_step() != 0 && m.row_step() < m.width() * m.point_step()) {
    reason = "row_step_too_small";
    return false;
  }
  if (!has_xyz_fields(m.fields())) { reason = "missing_xyz_fields"; return false; }

  const builtin_interfaces::msg::Time& st = m.header().stamp();
  if (st.sec() == 0 && st.nanosec() == 0) { reason = "stamp_zero"; return false; }

  return true;
}

void on_point_cloud(const sensor_msgs::msg::PointCloud2* msg)
{
  if (msg == nullptr) { return; }

  const std::chrono::steady_clock::time_point now_steady =
      std::chrono::steady_clock::now();
  const double now_wall = wall_now();

  std::lock_guard<std::mutex> lk(g_stats.m);

  const bool first_frame = (g_stats.frames == 0);
  g_stats.frames++;
  g_stats.last_rx = now_steady;
  if (first_frame) { g_stats.first_rx = now_steady; }

  // raw layout values of the last received frame (never rewritten)
  g_stats.width        = msg->width();
  g_stats.height       = msg->height();
  g_stats.point_step   = msg->point_step();
  g_stats.row_step     = msg->row_step();
  g_stats.data_bytes   = msg->data().size();
  g_stats.is_bigendian = msg->is_bigendian();
  g_stats.is_dense     = msg->is_dense();

  if (first_frame) {
    const std::vector<sensor_msgs::msg::PointField>& fs = msg->fields();
    g_stats.field_count = static_cast<int>(fs.size());
    g_stats.first_fields.clear();
    for (size_t i = 0; i < fs.size(); ++i) {
      FieldInfo f;
      f.name     = fs[i].name();
      f.offset   = fs[i].offset();
      f.datatype = fs[i].datatype();
      g_stats.first_fields.push_back(f);
    }
    g_stats.xyz_present = has_xyz_fields(fs);
  }

  std::string reason;
  if (frame_is_valid(*msg, reason)) {
    g_stats.valid_frames++;
    g_stats.latest_valid = true;
    g_stats.last_valid_rx = now_steady;
    g_stats.has_last_valid_rx = true;
    g_stats.last_valid_frame_id = msg->header().frame_id();
    g_stats.last_valid_sec     = msg->header().stamp().sec();
    g_stats.last_valid_nanosec = msg->header().stamp().nanosec();

    const double src_age =
        now_wall -
        (g_stats.last_valid_sec + g_stats.last_valid_nanosec * 1e-9);
    g_stats.age_last = src_age;
    if (!g_stats.has_age || src_age < g_stats.age_min) { g_stats.age_min = src_age; }
    if (!g_stats.has_age || src_age > g_stats.age_max) { g_stats.age_max = src_age; }
    g_stats.has_age = true;
    g_stats.last_invalid_reason.clear();
  } else {
    g_stats.latest_valid = false;
    g_stats.age_last = 0.0;              // no valid age for an invalid frame
    g_stats.has_age = g_stats.valid_frames > 0;   // min/max still valid
    g_stats.last_invalid_reason = reason;
  }
}

std::string fields_json()
{
  std::ostringstream os;
  os << "{\"count\":" << g_stats.field_count
     << ",\"xyz_present\":" << (g_stats.xyz_present ? "true" : "false")
     << ",\"list\":[";
  for (size_t i = 0; i < g_stats.first_fields.size(); ++i) {
    if (i != 0) { os << ","; }
    os << "{\"name\":\"" << json_escape(g_stats.first_fields[i].name)
       << "\",\"offset\":" << g_stats.first_fields[i].offset
       << ",\"datatype\":" << static_cast<int>(g_stats.first_fields[i].datatype)
       << "}";
  }
  os << "]}";
  return os.str();
}

std::string build_report(
    int seconds_requested, double seconds_used, bool interrupted,
    int matched_count, bool manager_ok, int exit_code)
{
  const std::chrono::steady_clock::time_point now_steady =
      std::chrono::steady_clock::now();
  const double now_wall = wall_now();

  std::string result;
  std::ostringstream os;
  uint64_t frames = 0, valid_frames = 0;
  bool latest_valid = false;
  bool has_last_valid_rx = false, has_age = false;
  double first_rx_s = 0.0, last_rx_s = 0.0;
  double age_min = 0.0, age_max = 0.0, age_last = 0.0;
  double src_age_exit = 0.0, recv_age_exit = 0.0;
  bool fresh = false;
  std::string fresh_detail;

  {
    std::lock_guard<std::mutex> lk(g_stats.m);
    frames       = g_stats.frames;
    valid_frames = g_stats.valid_frames;
    latest_valid = g_stats.latest_valid;
    has_last_valid_rx = g_stats.has_last_valid_rx;
    has_age  = g_stats.has_age;
    age_min  = g_stats.age_min;
    age_max  = g_stats.age_max;
    age_last = g_stats.age_last;
    if (g_stats.frames > 0) {
      first_rx_s = std::chrono::duration<double>(g_stats.first_rx -
                                                 g_start_steady).count();
      last_rx_s  = std::chrono::duration<double>(g_stats.last_rx -
                                                 g_start_steady).count();
    }
    if (g_stats.has_last_valid_rx) {
      recv_age_exit = std::chrono::duration<double>(now_steady -
          g_stats.last_valid_rx).count();
      src_age_exit = now_wall -
          (g_stats.last_valid_sec + g_stats.last_valid_nanosec * 1e-9);
      fresh = g_stats.latest_valid &&
              recv_age_exit <= kRecvFreshMax &&
              src_age_exit <= kFreshMaxAge &&
              src_age_exit >= -kFreshMaxFuture;
      std::ostringstream d;
      d << "recv_age=" << recv_age_exit
        << " src_age=" << src_age_exit
        << " latest_valid=" << (g_stats.latest_valid ? "true" : "false");
      fresh_detail = d.str();
    }
  }

  const double hz =
      (frames >= 2 && last_rx_s > first_rx_s)
          ? static_cast<double>(frames - 1) / (last_rx_s - first_rx_s)
          : 0.0;

  if (interrupted) {
    result = "INTERRUPTED";               // not a full success by definition
  } else if (fresh) {
    result = "SUCCESS";
  } else {
    result = "NO_FRESH_FRAME";
  }

  os << "{\"task\":\"NEW-07\""
     << ",\"probe\":\"drdds_cloud_probe\""
     << ",\"uid\":"     << static_cast<long>(getuid())
     << ",\"euid\":"    << static_cast<long>(geteuid())
     << ",\"transport_api\":\"DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType>"
        " use_shm=true; subscription-only (no publisher created by subscriber ctor,"
        " verified against drdds_core.h member list)\""
     << ",\"manager_api\":\"DrDDSManager::Init(0) default network path; "
        "DrDDSManager::Delete() after subscriber destruction\""
     << ",\"topic\":\"" << json_escape(kTopic) << "\""
     << ",\"domain\":"  << kDomain
     << ",\"prefix\":\"" << json_escape(kPrefix) << "\""
     << ",\"use_shm\":" << (kUseShm ? "true" : "false")
     << ",\"seconds_requested\":" << seconds_requested
     << ",\"seconds_used\":" << num_as_json(seconds_used)
     << ",\"manager_ok\":" << (manager_ok ? "true" : "false")
     << ",\"matched_count\":" << matched_count
     << ",\"result\":\"" << result << "\""
     << ",\"interrupted\":" << (interrupted ? "true" : "false")
     << ",\"frames\":"  << frames
     << ",\"valid_frames\":" << valid_frames
     << ",\"latest_valid\":" << (latest_valid ? "true" : "false")
     << ",\"first_rx_s\":" << (frames > 0 ? num_as_json(first_rx_s) : std::string("null"))
     << ",\"last_rx_s\":"  << (frames > 0 ? num_as_json(last_rx_s)  : std::string("null"))
     << ",\"hz\":"         << (frames >= 2 ? num_as_json(hz) : std::string("null"))
     << ",\"last_frame\":{\"width\":" << g_stats.width
     << ",\"height\":" << g_stats.height
     << ",\"point_step\":" << g_stats.point_step
     << ",\"row_step\":" << g_stats.row_step
     << ",\"data_bytes\":" << g_stats.data_bytes
     << ",\"is_bigendian\":" << (g_stats.is_bigendian ? "true" : "false")
     << ",\"is_dense\":" << (g_stats.is_dense ? "true" : "false") << "}"
     << ",\"last_valid_frame\":{\"frame_id\":\""
        << json_escape(g_stats.last_valid_frame_id)
     << "\",\"stamp_sec\":" << g_stats.last_valid_sec
     << ",\"stamp_nanosec\":" << g_stats.last_valid_nanosec << "}"
     << ",\"fields\":" << fields_json()
     << ",\"age_min_s\":" << (has_age ? num_as_json(age_min) : std::string("null"))
     << ",\"age_max_s\":" << (has_age ? num_as_json(age_max) : std::string("null"))
     << ",\"age_last_s\":" << ((has_age && latest_valid) ? num_as_json(age_last)
                                                         : std::string("null"))
     << ",\"source_age_at_exit_s\":" << (has_last_valid_rx ? num_as_json(src_age_exit)
                                                           : std::string("null"))
     << ",\"recv_age_at_exit_s\":" << (has_last_valid_rx ? num_as_json(recv_age_exit)
                                                         : std::string("null"))
     << ",\"fresh\":" << (fresh ? "true" : "false")
     << ",\"fresh_detail\":\"" << json_escape(fresh_detail) << "\""
     << ",\"last_invalid_reason\":\"" << json_escape(g_stats.last_invalid_reason) << "\""
     << ",\"exit_code\":" << exit_code
     << ",\"shm_note\":\"use_shm=true creates FastDDS SHM segment(s) in /dev/shm "
        "(observed vendor size ~50MB data segment); no shm files are ever unlinked "
        "by this process; before/after SHM observation is external (shell)\""
     << ",\"cleanup\":\"DrDDSSubscriber destroyed first (RAII scope), then "
        "DrDDSManager::Delete()\""
     << "}";
  return os.str();
}

void print_banner()
{
  std::cout << "uid=" << static_cast<long>(getuid())
            << " euid=" << static_cast<long>(geteuid())
            << " transport=DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType>"
            << " topic=" << kTopic
            << " full_topic_rt=" << kPrefix << kTopic
            << " domain=" << kDomain
            << " use_shm=" << (kUseShm ? "true" : "false")
            << " mode=subscription-only" << std::endl;
}

}  // namespace

// ---------------------------------------------------------------------- main
int main(int argc, char** argv)
{
  g_start_steady = std::chrono::steady_clock::now();

  int seconds = kDefaultSeconds;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--seconds" && i + 1 < argc) {
      seconds = std::atoi(argv[++i]);
    } else if (a.rfind("--seconds=", 0) == 0) {
      seconds = std::atoi(a.c_str() + 10);
    } else if (a == "--local" && i + 1 < argc) {
      // accepted only for interface compatibility; forced to "true"
      if (std::string(argv[++i]) != "true") {
        std::cout << "{\"task\":\"NEW-07\",\"result\":\"ABORT\",\"exit_code\":2,"
                     "\"reason\":\"--local only supports true\"}" << std::endl;
        return 2;
      }
    } else if (a.rfind("--local=", 0) == 0) {
      if (a.substr(8) != "true") {
        std::cout << "{\"task\":\"NEW-07\",\"result\":\"ABORT\",\"exit_code\":2,"
                     "\"reason\":\"--local only supports true\"}" << std::endl;
        return 2;
      }
    } else {
      std::cout << "{\"task\":\"NEW-07\",\"result\":\"ABORT\",\"exit_code\":2,"
                   "\"reason\":\"unknown argument: " << json_escape(a) << "\"}"
                << std::endl;
      return 2;
    }
  }

  if (seconds < kMinSeconds || seconds > kMaxSeconds) {
    std::cout << "{\"task\":\"NEW-07\",\"result\":\"ABORT\",\"exit_code\":2,"
                 "\"reason\":\"--seconds out of range [1,20]\"}" << std::endl;
    return 2;
  }

  // Do NOT load any external (possibly 500MB-SHM) FastDDS XML profiles.
  if (std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE") != nullptr ||
      std::getenv("FASTDDS_DEFAULT_PROFILES_FILE") != nullptr) {
    std::cout << "{\"task\":\"NEW-07\",\"result\":\"ABORT\",\"exit_code\":2,"
                 "\"reason\":\"FASTRTPS/FASTDDS_DEFAULT_PROFILES_FILE must be "
                 "unset for this probe\"}" << std::endl;
    return 2;
  }

  // SIGINT/SIGTERM -> flag only; graceful RAII release, never _exit/abort.
  struct sigaction sa;
  std::memset(&sa, 0, sizeof(sa));
  sa.sa_sigaction = [](int, siginfo_t*, void*) { g_quit = 1; };
  sa.sa_flags = SA_SIGINFO;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT,  &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  print_banner();

  int  exit_code    = 2;
  bool interrupted  = false;
  bool manager_ok   = false;
  int  matched_count = -1;

  try {
    DrDDSManager::Init(0);      // default vendor participant path, domain 0
    manager_ok = true;

    {
      DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType> sub(
          &on_point_cloud, kTopic, kDomain, kPrefix, kUseShm);

      const std::chrono::steady_clock::time_point deadline =
          std::chrono::steady_clock::now() +
          std::chrono::seconds(seconds);

      // bounded wait, at most 50 ms granularity, signal-flag responsive
      while (!g_quit && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      interrupted = (g_quit != 0);

      matched_count = sub.GetMatchedCount();
    }  // subscriber fully destroyed here BEFORE manager Delete

    DrDDSManager::Delete();

    const double seconds_used = steady_since_start();

    // decide exit code FIRST, then emit the report with the real code
    {
      std::lock_guard<std::mutex> lk(g_stats.m);
      const bool fresh_now =
          g_stats.latest_valid &&
          g_stats.has_last_valid_rx &&
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - g_stats.last_valid_rx).count()
              <= kRecvFreshMax &&
          g_stats.valid_frames > 0 &&
          (wall_now() -
              (g_stats.last_valid_sec + g_stats.last_valid_nanosec * 1e-9))
              <= kFreshMaxAge &&
          (wall_now() -
              (g_stats.last_valid_sec + g_stats.last_valid_nanosec * 1e-9))
              >= -kFreshMaxFuture;
      if (interrupted) {
        exit_code = 1;                 // interrupted = not a full success
      } else if (fresh_now) {
        exit_code = 0;
      } else {
        exit_code = 1;
      }
    }

    const std::string report = build_report(
        seconds, seconds_used, interrupted, matched_count, manager_ok,
        exit_code);

    std::cout << report << std::endl;
    return exit_code;
  } catch (const std::exception& e) {
    DrDDSManager::Delete();            // RAII: paired cleanup on failure path
    std::cout << "{\"task\":\"NEW-07\",\"result\":\"ABORT\",\"exit_code\":2,"
                 "\"interrupted\":" << (interrupted ? "true" : "false")
              << ",\"reason\":\"exception: "
              << json_escape(e.what()) << "\"}" << std::endl;
    return 2;
  } catch (...) {
    DrDDSManager::Delete();
    std::cout << "{\"task\":\"NEW-07\",\"result\":\"ABORT\",\"exit_code\":2,"
                 "\"interrupted\":" << (interrupted ? "true" : "false")
              << ",\"reason\":\"unknown exception\"}" << std::endl;
    return 2;
  }
}
