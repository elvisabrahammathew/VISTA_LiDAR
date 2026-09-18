//! Windows-specific TCP stream and thread-priority configuration.

#![allow(unsafe_code)]

use std::{
    ffi::c_void,
    io,
    net::TcpStream,
    os::windows::ffi::OsStrExt,
    path::Path,
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

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct FileTime {
    low: u32,
    high: u32,
}

#[repr(C)]
struct ProcessMemoryCountersEx {
    size: u32,
    page_fault_count: u32,
    peak_working_set_size: usize,
    working_set_size: usize,
    quota_peak_paged_pool_usage: usize,
    quota_paged_pool_usage: usize,
    quota_peak_non_paged_pool_usage: usize,
    quota_non_paged_pool_usage: usize,
    pagefile_usage: usize,
    peak_pagefile_usage: usize,
    private_usage: usize,
}

impl Default for ProcessMemoryCountersEx {
    fn default() -> Self {
        Self {
            size: std::mem::size_of::<Self>() as u32,
            page_fault_count: 0,
            peak_working_set_size: 0,
            working_set_size: 0,
            quota_peak_paged_pool_usage: 0,
            quota_paged_pool_usage: 0,
            quota_peak_non_paged_pool_usage: 0,
            quota_non_paged_pool_usage: 0,
            pagefile_usage: 0,
            peak_pagefile_usage: 0,
            private_usage: 0,
        }
    }
}

#[repr(C)]
struct MemoryStatusEx {
    length: u32,
    memory_load: u32,
    total_physical: u64,
    available_physical: u64,
    total_page_file: u64,
    available_page_file: u64,
    total_virtual: u64,
    available_virtual: u64,
    available_extended_virtual: u64,
}

impl Default for MemoryStatusEx {
    fn default() -> Self {
        Self {
            length: std::mem::size_of::<Self>() as u32,
            memory_load: 0,
            total_physical: 0,
            available_physical: 0,
            total_page_file: 0,
            available_page_file: 0,
            total_virtual: 0,
            available_virtual: 0,
            available_extended_virtual: 0,
        }
    }
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
    fn GetSystemTimes(idle: *mut FileTime, kernel: *mut FileTime, user: *mut FileTime) -> i32;
    fn GetCurrentProcess() -> *mut c_void;
    fn GlobalMemoryStatusEx(status: *mut MemoryStatusEx) -> i32;
    fn GetDiskFreeSpaceExW(
        directory: *const u16,
        available_to_caller: *mut u64,
        total_bytes: *mut u64,
        total_free_bytes: *mut u64,
    ) -> i32;
}

#[link(name = "psapi")]
unsafe extern "system" {
    fn GetProcessMemoryInfo(
        process: *mut c_void,
        counters: *mut ProcessMemoryCountersEx,
        size: u32,
    ) -> i32;
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

fn file_time_value(value: FileTime) -> u64 {
    (u64::from(value.high) << 32) | u64::from(value.low)
}

/// Stateful Windows CPU sampler based on GetSystemTimes deltas.
#[derive(Default)]
pub struct CpuSampler {
    initialized: bool,
    previous_idle: u64,
    previous_total: u64,
}

impl CpuSampler {
    pub fn sample(&mut self) -> f64 {
        let mut idle = FileTime::default();
        let mut kernel = FileTime::default();
        let mut user = FileTime::default();
        // SAFETY: all three pointers reference writable FILETIME-compatible values.
        if unsafe { GetSystemTimes(&mut idle, &mut kernel, &mut user) } == 0 {
            return 0.0;
        }
        let idle = file_time_value(idle);
        let total = file_time_value(kernel) + file_time_value(user);
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

pub fn process_memory_mb() -> f64 {
    let mut counters = ProcessMemoryCountersEx::default();
    // SAFETY: GetCurrentProcess returns a valid pseudo-handle and counters has
    // the documented PROCESS_MEMORY_COUNTERS_EX layout and size.
    let success = unsafe {
        GetProcessMemoryInfo(
            GetCurrentProcess(),
            &mut counters,
            std::mem::size_of::<ProcessMemoryCountersEx>() as u32,
        )
    };
    if success == 0 {
        0.0
    } else {
        counters.working_set_size as f64 / (1024.0 * 1024.0)
    }
}

pub fn system_memory_percent() -> f64 {
    let mut status = MemoryStatusEx::default();
    // SAFETY: status is writable and its length field identifies the layout.
    if unsafe { GlobalMemoryStatusEx(&mut status) } == 0 {
        0.0
    } else {
        f64::from(status.memory_load)
    }
}

pub fn disk_free_gb(path: &Path) -> f64 {
    let mut wide = path.as_os_str().encode_wide().collect::<Vec<_>>();
    wide.push(0);
    let mut available = 0_u64;
    // SAFETY: wide is null-terminated and available points to writable storage.
    if unsafe {
        GetDiskFreeSpaceExW(
            wide.as_ptr(),
            &mut available,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    } == 0
    {
        0.0
    } else {
        available as f64 / (1024.0 * 1024.0 * 1024.0)
    }
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
