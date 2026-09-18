//! Linux-specific TCP stream and thread-priority configuration.

#![allow(unsafe_code)]

use std::{
    ffi::{c_char, c_int, c_long, c_ulong, CString},
    fs, io,
    net::TcpStream,
    os::unix::ffi::OsStrExt,
    path::Path,
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};

use super::threading::{NativeThreadPriority, ThreadPriority};

pub const PLATFORM_NAME: &str = "Linux";

pub fn configure_tcp_stream(stream: &TcpStream, read_timeout: Duration) -> io::Result<()> {
    // Reduce latency and prevent a stalled sensor from blocking forever.
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(read_timeout))
}

const PRIO_PROCESS: c_int = 0;
const NICE_LEVELS: [c_int; 5] = [0, 2, 5, 10, 15];
const SIGINT: c_int = 2;
const SIGTERM: c_int = 15;
const SIG_ERR: usize = usize::MAX;

static SHUTDOWN_SIGNAL_RECEIVED: AtomicBool = AtomicBool::new(false);

#[repr(C)]
struct BrokenDownTime {
    second: c_int,
    minute: c_int,
    hour: c_int,
    day: c_int,
    month: c_int,
    year: c_int,
    weekday: c_int,
    year_day: c_int,
    is_dst: c_int,
    gmtoff: c_long,
    zone: *const i8,
}

#[repr(C)]
struct StatVfs {
    block_size: c_ulong,
    fragment_size: c_ulong,
    blocks: u64,
    blocks_free: u64,
    blocks_available: u64,
    files: u64,
    files_free: u64,
    files_available: u64,
    filesystem_id: c_ulong,
    mount_flags: c_ulong,
    maximum_name_length: c_ulong,
    spare: [c_int; 6],
}

unsafe extern "C" {
    fn setpriority(which: c_int, who: u32, priority: c_int) -> c_int;
    fn getpriority(which: c_int, who: u32) -> c_int;
    fn signal(signal: c_int, handler: usize) -> usize;
    fn time(output: *mut c_long) -> c_long;
    fn localtime_r(input: *const c_long, output: *mut BrokenDownTime) -> *mut BrokenDownTime;
    fn statvfs(path: *const c_char, output: *mut StatVfs) -> c_int;
}

extern "C" fn record_shutdown_signal(_: c_int) {
    SHUTDOWN_SIGNAL_RECEIVED.store(true, Ordering::Release);
}

/// Installs SIGINT/SIGTERM handlers used by the main 24/7 wait loop.
pub fn install_shutdown_signal_handlers() -> io::Result<()> {
    SHUTDOWN_SIGNAL_RECEIVED.store(false, Ordering::Release);
    let handler = record_shutdown_signal as *const () as usize;
    // SAFETY: the handler has C signal ABI and only sets a lock-free atomic.
    if unsafe { signal(SIGINT, handler) } == SIG_ERR
        || unsafe { signal(SIGTERM, handler) } == SIG_ERR
    {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

pub fn shutdown_signal_received() -> bool {
    SHUTDOWN_SIGNAL_RECEIVED.load(Ordering::Acquire)
}

/// Formats the current Linux local time for RAW and PCD filenames.
pub fn current_log_timestamp() -> io::Result<String> {
    // SAFETY: null asks libc to return the current epoch time directly.
    let raw_time = unsafe { time(std::ptr::null_mut()) };
    let mut local_time = BrokenDownTime {
        second: 0,
        minute: 0,
        hour: 0,
        day: 0,
        month: 0,
        year: 0,
        weekday: 0,
        year_day: 0,
        is_dst: 0,
        gmtoff: 0,
        zone: std::ptr::null(),
    };
    // SAFETY: both pointers remain valid for the duration of localtime_r.
    if unsafe { localtime_r(&raw_time, &mut local_time) }.is_null() {
        return Err(io::Error::last_os_error());
    }
    Ok(format!(
        "{:04}{:02}{:02}_{:02}{:02}{:02}",
        local_time.year + 1900,
        local_time.month + 1,
        local_time.day,
        local_time.hour,
        local_time.minute,
        local_time.second
    ))
}

/// Stateful Linux CPU sampler based on /proc/stat deltas.
#[derive(Default)]
pub struct CpuSampler {
    initialized: bool,
    previous_idle: u64,
    previous_total: u64,
}

impl CpuSampler {
    pub fn sample(&mut self) -> f64 {
        let Some((idle, total)) = read_cpu_times() else {
            return 0.0;
        };
        let percent = if self.initialized && total > self.previous_total {
            let total_delta = total - self.previous_total;
            let idle_delta = idle.saturating_sub(self.previous_idle);
            100.0 * total_delta.saturating_sub(idle_delta) as f64 / total_delta as f64
        } else {
            0.0
        };
        self.initialized = true;
        self.previous_idle = idle;
        self.previous_total = total;
        percent.clamp(0.0, 100.0)
    }
}

fn read_cpu_times() -> Option<(u64, u64)> {
    let contents = fs::read_to_string("/proc/stat").ok()?;
    let mut fields = contents.lines().next()?.split_whitespace();
    if fields.next()? != "cpu" {
        return None;
    }
    let values = fields
        .take(8)
        .map(str::parse::<u64>)
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    if values.len() < 5 {
        return None;
    }
    Some((values[3] + values[4], values.iter().sum()))
}

pub fn process_memory_mb() -> f64 {
    fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|contents| {
            contents.lines().find_map(|line| {
                line.strip_prefix("VmRSS:")
                    .and_then(|value| value.split_whitespace().next())
                    .and_then(|value| value.parse::<f64>().ok())
            })
        })
        .map_or(0.0, |kib| kib / 1024.0)
}

pub fn system_memory_percent() -> f64 {
    let Ok(contents) = fs::read_to_string("/proc/meminfo") else {
        return 0.0;
    };
    let mut total = 0.0;
    let mut available = 0.0;
    for line in contents.lines() {
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("MemTotal:") => total = fields.next().and_then(|v| v.parse().ok()).unwrap_or(0.0),
            Some("MemAvailable:") => {
                available = fields.next().and_then(|v| v.parse().ok()).unwrap_or(0.0)
            }
            _ => {}
        }
    }
    if total > 0.0 {
        100.0 * (total - available) / total
    } else {
        0.0
    }
}

pub fn disk_free_gb(path: &Path) -> f64 {
    let Ok(path) = CString::new(path.as_os_str().as_bytes()) else {
        return 0.0;
    };
    let mut output = std::mem::MaybeUninit::<StatVfs>::zeroed();
    // SAFETY: path is null-terminated and output points to writable statvfs storage.
    if unsafe { statvfs(path.as_ptr(), output.as_mut_ptr()) } != 0 {
        return 0.0;
    }
    // SAFETY: statvfs returned success and initialized the complete structure.
    let output = unsafe { output.assume_init() };
    output.blocks_available as f64 * output.fragment_size as f64 / (1024.0 * 1024.0 * 1024.0)
}

/// Maps project priorities 1..=5 to unprivileged SCHED_OTHER nice values.
pub(crate) fn apply_current_thread_priority(
    priority: ThreadPriority,
) -> io::Result<NativeThreadPriority> {
    let nice_value = NICE_LEVELS[usize::from(priority.level() - 1)];

    // Linux/NPTL treats nice as a per-thread property. `who = 0` selects the
    // calling thread, and non-negative values do not require CAP_SYS_NICE.
    // SAFETY: arguments are within the documented setpriority ranges.
    if unsafe { setpriority(PRIO_PROCESS, 0, nice_value) } != 0 {
        return Err(io::Error::last_os_error());
    }

    // SAFETY: this only queries the calling thread's SCHED_OTHER nice value.
    let applied = unsafe { getpriority(PRIO_PROCESS, 0) };
    if applied != nice_value {
        return Err(io::Error::other(format!(
            "Linux applied nice {applied}, expected {nice_value}"
        )));
    }

    Ok(NativeThreadPriority {
        policy: "SCHED_OTHER",
        value: applied,
    })
}
