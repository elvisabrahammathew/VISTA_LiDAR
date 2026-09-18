//! Focused tests for portable monitoring defaults.

use super::*;

#[test]
fn default_monitor_interval_is_positive() {
    let config = SystemMonitorConfig::default();
    assert!(!config.sample_interval.is_zero());
    assert!(!config.data_root.as_os_str().is_empty());
}

#[test]
fn power_sample_has_requested_timestamp() {
    let sample = power_thermal_sample(123);
    assert_eq!(sample.timestamp_ns, 123);
}

#[test]
fn host_metrics_return_sensible_values() {
    let mut cpu = CpuSampler::default();
    let first = cpu.sample();
    std::thread::sleep(Duration::from_millis(20));
    let second = cpu.sample();
    assert!((0.0..=100.0).contains(&first));
    assert!((0.0..=100.0).contains(&second));
    assert!(process_memory_mb() > 0.0);
    assert!((0.0..=100.0).contains(&system_memory_percent()));
    assert!(disk_free_gb(&std::env::current_dir().unwrap()) > 0.0);
}
