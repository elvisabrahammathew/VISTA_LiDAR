//! Linux-specific TCP stream and thread-priority configuration.

#![allow(unsafe_code)]

use std::{ffi::c_int, io, net::TcpStream, time::Duration};

use super::threading::{NativeThreadPriority, ThreadPriority};

pub const PLATFORM_NAME: &str = "Linux";

pub fn configure_tcp_stream(stream: &TcpStream) -> io::Result<()> {
    // Reduce latency and prevent a stalled sensor from blocking forever.
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(Duration::from_secs(5)))
}

const PRIO_PROCESS: c_int = 0;
const NICE_LEVELS: [c_int; 5] = [0, 2, 5, 10, 15];

unsafe extern "C" {
    fn setpriority(which: c_int, who: u32, priority: c_int) -> c_int;
    fn getpriority(which: c_int, who: u32) -> c_int;
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
