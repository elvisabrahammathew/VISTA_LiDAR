#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointXYZIRT {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: u8,
    pub ring: u8,
    pub return_id: u8,
    pub timestamp_ns: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PointCloudFrame {
    pub timestamp_ns: u64,
    pub points: Vec<PointXYZIRT>,
}

impl PointCloudFrame {
    pub fn new(timestamp_ns: u64, points: Vec<PointXYZIRT>) -> Self {
        Self {
            timestamp_ns,
            points,
        }
    }
}
