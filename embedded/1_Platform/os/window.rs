//! Windows-specific TCP stream and thread-priority configuration.

#![allow(unsafe_code)]

use std::{ffi::c_void, io, net::TcpStream, time::Duration};

use super::threading::{NativeThreadPriority, ThreadPriority};

pub const PLATFORM_NAME: &str = "Windows";

pub fn configure_tcp_stream(stream: &TcpStream) -> io::Result<()> {
    // Reduce latency and prevent a stalled sensor from blocking forever.
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(Duration::from_secs(5)))
}

const THREAD_PRIORITY_LEVELS: [i32; 5] = [2, 1, 0, -1, -2];
const THREAD_PRIORITY_ERROR_RETURN: i32 = i32::MAX;

#[link(name = "kernel32")]
unsafe extern "system" {
    fn GetCurrentThread() -> *mut c_void;
    fn SetThreadPriority(thread: *mut c_void, priority: i32) -> i32;
    fn GetThreadPriority(thread: *mut c_void) -> i32;
}

/// Maps project priorities 1..=5 to Windows HIGHEST..LOWEST levels.
pub(crate) fn apply_current_thread_priority(
    priority: ThreadPriority,
) -> io::Result<NativeThreadPriority> {
    let native_value = THREAD_PRIORITY_LEVELS[usize::from(priority.level() - 1)];

    // SAFETY: GetCurrentThread returns a valid pseudo-handle for the calling
    // thread; it must not be closed and is used only during these calls.
    let thread = unsafe { GetCurrentThread() };
    // SAFETY: `thread` is the current-thread pseudo-handle and `native_value`
    // is one of the documented THREAD_PRIORITY_* integer values.
    if unsafe { SetThreadPriority(thread, native_value) } == 0 {
        return Err(io::Error::last_os_error());
    }

    // Read the value back so startup logs show what Windows actually applied.
    // SAFETY: the pseudo-handle remains valid throughout the current thread.
    let applied = unsafe { GetThreadPriority(thread) };
    if applied == THREAD_PRIORITY_ERROR_RETURN {
        return Err(io::Error::last_os_error());
    }

    Ok(NativeThreadPriority {
        policy: "NORMAL_PRIORITY_CLASS",
        value: applied,
    })
}
