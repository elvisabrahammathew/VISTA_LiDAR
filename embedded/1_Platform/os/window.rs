//! Windows-specific TCP stream and thread-priority configuration.

#![allow(unsafe_code)]

use std::{
    ffi::c_void,
    io,
    net::TcpStream,
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};

use super::threading::{NativeThreadPriority, ThreadPriority};

pub const PLATFORM_NAME: &str = "Windows";

pub fn configure_tcp_stream(stream: &TcpStream, read_timeout: Duration) -> io::Result<()> {
    // Reduce latency and prevent a stalled sensor from blocking forever.
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(read_timeout))
}

const THREAD_PRIORITY_LEVELS: [i32; 5] = [2, 1, 0, -1, -2];
const THREAD_PRIORITY_ERROR_RETURN: i32 = i32::MAX;
const CTRL_C_EVENT: u32 = 0;
const CTRL_BREAK_EVENT: u32 = 1;
const CTRL_CLOSE_EVENT: u32 = 2;
const CTRL_SHUTDOWN_EVENT: u32 = 6;

static SHUTDOWN_SIGNAL_RECEIVED: AtomicBool = AtomicBool::new(false);

#[repr(C)]
#[derive(Default)]
struct SystemTimeFields {
    year: u16,
    month: u16,
    day_of_week: u16,
    day: u16,
    hour: u16,
    minute: u16,
    second: u16,
    milliseconds: u16,
}

#[link(name = "kernel32")]
unsafe extern "system" {
    fn GetCurrentThread() -> *mut c_void;
    fn SetThreadPriority(thread: *mut c_void, priority: i32) -> i32;
    fn GetThreadPriority(thread: *mut c_void) -> i32;
    fn SetConsoleCtrlHandler(
        handler: Option<unsafe extern "system" fn(u32) -> i32>,
        add: i32,
    ) -> i32;
    fn GetLocalTime(system_time: *mut SystemTimeFields);
}

unsafe extern "system" fn record_shutdown_signal(control_type: u32) -> i32 {
    if matches!(
        control_type,
        CTRL_C_EVENT | CTRL_BREAK_EVENT | CTRL_CLOSE_EVENT | CTRL_SHUTDOWN_EVENT
    ) {
        SHUTDOWN_SIGNAL_RECEIVED.store(true, Ordering::Release);
        1
    } else {
        0
    }
}

/// Installs the console handler used by the main 24/7 wait loop.
pub fn install_shutdown_signal_handlers() -> io::Result<()> {
    SHUTDOWN_SIGNAL_RECEIVED.store(false, Ordering::Release);
    // SAFETY: the callback has the documented Windows console-handler ABI and
    // only sets a lock-free atomic flag.
    if unsafe { SetConsoleCtrlHandler(Some(record_shutdown_signal), 1) } == 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

pub fn shutdown_signal_received() -> bool {
    SHUTDOWN_SIGNAL_RECEIVED.load(Ordering::Acquire)
}

/// Formats the current Windows local time for RAW and PCD filenames.
pub fn current_log_timestamp() -> io::Result<String> {
    let mut value = SystemTimeFields::default();
    // SAFETY: `value` is writable and has the documented SYSTEMTIME layout.
    unsafe { GetLocalTime(&mut value) };
    if value.year == 0 || value.month == 0 || value.day == 0 {
        return Err(io::Error::other("Windows returned an invalid local time"));
    }
    Ok(format!(
        "{:04}{:02}{:02}_{:02}{:02}{:02}",
        value.year, value.month, value.day, value.hour, value.minute, value.second
    ))
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
