//! Unit tests for pointcloud_processing.rs preprocessing stages.

use super::*;

/// Creates a compact point value for preprocessing unit tests.
fn point(x: f32, y: f32, z: f32) -> PointXYZIRT {
    PointXYZIRT {
        x,
        y,
        z,
        intensity: 1,
        ring: 0,
        return_id: 0,
        timestamp_ns: 10,
    }
}

/// Creates one frame containing the supplied test points.
fn frame(points: Vec<PointXYZIRT>) -> PointCloudFrame {
    PointCloudFrame::new(10, points)
}

/// Verifies that invalid, near, far, and out-of-ROI points are removed.
#[test]
fn filters_invalid_range_and_roi_points() {
    let config = PreprocessingConfig {
        min_distance_m: 1.0,
        max_distance_m: 10.0,
        region_of_interest: Some(AxisAlignedRoi {
            min_x: 0.0,
            max_x: 10.0,
            min_y: -2.0,
            max_y: 2.0,
            min_z: -2.0,
            max_z: 2.0,
        }),
        voxel_size_m: None,
        ground_removal: None,
    };
    let input = frame(vec![
        point(2.0, 0.0, 0.0),
        point(0.1, 0.0, 0.0),
        point(20.0, 0.0, 0.0),
        point(2.0, 3.0, 0.0),
        point(f32::NAN, 0.0, 0.0),
    ]);

    let output = preprocess_point_cloud(input, &config).unwrap();
    assert_eq!(output.points, vec![point(2.0, 0.0, 0.0)]);
}

/// Verifies that only the first point in each voxel is retained.
#[test]
fn downsamples_points_in_the_same_voxel() {
    let config = PreprocessingConfig {
        min_distance_m: 0.0,
        max_distance_m: 10.0,
        region_of_interest: None,
        voxel_size_m: Some(1.0),
        ground_removal: None,
    };
    let input = frame(vec![
        point(1.1, 2.1, 3.1),
        point(1.8, 2.8, 3.8),
        point(2.1, 2.1, 3.1),
    ]);

    let output = preprocess_point_cloud(input, &config).unwrap();
    assert_eq!(output.points.len(), 2);
    assert_eq!(output.points[0], point(1.1, 2.1, 3.1));
    assert_eq!(output.points[1], point(2.1, 2.1, 3.1));
}

/// Verifies fixed-height ground removal without removing points above the band.
#[test]
fn removes_points_near_the_ground_height() {
    let config = PreprocessingConfig {
        min_distance_m: 0.0,
        max_distance_m: 10.0,
        region_of_interest: None,
        voxel_size_m: None,
        ground_removal: Some(GroundRemovalConfig {
            ground_height_m: -1.5,
            tolerance_m: 0.15,
        }),
    };
    let input = frame(vec![
        point(2.0, 0.0, -1.50),
        point(2.0, 0.0, -1.40),
        point(2.0, 0.0, -0.50),
    ]);

    let output = preprocess_point_cloud(input, &config).unwrap();
    assert_eq!(output.points, vec![point(2.0, 0.0, -0.50)]);
}

/// Verifies that unusable voxel sizes are rejected instead of silently ignored.
#[test]
fn rejects_invalid_voxel_size() {
    let config = PreprocessingConfig {
        voxel_size_m: Some(0.0),
        ..PreprocessingConfig::default()
    };

    assert!(preprocess_point_cloud(frame(Vec::new()), &config).is_err());
}
