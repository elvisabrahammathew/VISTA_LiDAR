//! Message envelope used only by LiDAR topics.

/// Carries one LiDAR payload together with ordering and timing metadata.
///
/// Radar and other sensor families intentionally define their own message
/// structures instead of depending on this LiDAR-specific type.
#[derive(Debug)]
pub struct LidarMessage<T> {
    /// Identifies the LiDAR instance that produced this message.
    pub lidar_id: String,
    /// Increases for every RAW packet read during one capture session.
    pub sequence: u64,
    /// Timestamp supplied by the LiDAR, when the device protocol provides one.
    pub sensor_timestamp_ns: Option<u64>,
    /// Host wall-clock timestamp recorded immediately after receiving the packet.
    pub received_timestamp_ns: u64,
    /// Strongly typed RAW packet or point-cloud frame carried by the topic.
    pub payload: T,
}

impl<T> LidarMessage<T> {
    /// Wraps one LiDAR payload without interpreting or synchronizing timestamps.
    pub fn new(
        lidar_id: impl Into<String>,
        sequence: u64,
        sensor_timestamp_ns: Option<u64>,
        received_timestamp_ns: u64,
        payload: T,
    ) -> Self {
        Self {
            lidar_id: lidar_id.into(),
            sequence,
            sensor_timestamp_ns,
            received_timestamp_ns,
            payload,
        }
    }
}
