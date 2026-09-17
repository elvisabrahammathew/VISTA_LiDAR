//! Linux-specific TCP stream and thread-priority configuration.

#![allow(unsafe_code)]

use std::{
    ffi::{c_int, c_long},
    io,
    net::TcpStream,
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

unsafe extern "C" {
    fn setpriority(which: c_int, who: u32, priority: c_int) -> c_int;
    fn getpriority(which: c_int, who: u32) -> c_int;
    fn signal(signal: c_int, handler: usize) -> usize;
    fn time(output: *mut c_long) -> c_long;
    fn localtime_r(input: *const c_long, output: *mut BrokenDownTime) -> *mut BrokenDownTime;
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
