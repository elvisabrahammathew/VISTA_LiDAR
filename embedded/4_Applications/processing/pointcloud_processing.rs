//! Sensor-neutral point-cloud preprocessing performed before object detection.

use std::{
    collections::HashSet,
    io::{self, ErrorKind},
};

use crate::models::pointcloud::{PointCloudFrame, PointXYZIRT};

/// Describes an axis-aligned region of interest in sensor coordinates.
#[derive(Debug, Clone, Copy)]
pub struct AxisAlignedRoi {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
    pub min_z: f32,
    pub max_z: f32,
}

impl AxisAlignedRoi {
    /// Reports whether a point is inside every configured ROI boundary.
    fn contains(&self, point: &PointXYZIRT) -> bool {
        point.x >= self.min_x
            && point.x <= self.max_x
            && point.y >= self.min_y
            && point.y <= self.max_y
            && point.z >= self.min_z
            && point.z <= self.max_z
    }

    /// Checks that every ROI axis has finite and correctly ordered bounds.
    fn validate(&self) -> io::Result<()> {
        let values = [
            self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z,
        ];
        if values.iter().any(|value| !value.is_finite()) {
            return Err(invalid_input("ROI bounds must be finite"));
        }
        if self.min_x > self.max_x || self.min_y > self.max_y || self.min_z > self.max_z {
            return Err(invalid_input(
                "each ROI minimum must be less than or equal to its maximum",
            ));
        }
        Ok(())
    }
}

/// Removes points close to a known horizontal ground height.
#[derive(Debug, Clone, Copy)]
pub struct GroundRemovalConfig {
    /// Ground Z coordinate relative to the LiDAR, in meters.
    pub ground_height_m: f32,
    /// Half-width of the removed band around the ground height, in meters.
    pub tolerance_m: f32,
}

/// Controls each stage of sensor-neutral point-cloud preprocessing.
#[derive(Debug, Clone, Copy)]
pub struct PreprocessingConfig {
    /// Closest accepted radial distance from the LiDAR, in meters.
    pub min_distance_m: f32,
    /// Farthest accepted radial distance from the LiDAR, in meters.
    pub max_distance_m: f32,
    /// Optional 3D region retained for later processing.
    pub region_of_interest: Option<AxisAlignedRoi>,
    /// Optional voxel side length; one input point is retained per occupied voxel.
    pub voxel_size_m: Option<f32>,
    /// Optional fixed-height ground band removal.
    pub ground_removal: Option<GroundRemovalConfig>,
}

impl Default for PreprocessingConfig {
    /// Supplies conservative defaults without assuming sensor mounting height.
    fn default() -> Self {
        Self {
            min_distance_m: 0.1,
            max_distance_m: 200.0,
            region_of_interest: None,
            voxel_size_m: Some(0.05),
            ground_removal: None,
        }
    }
}

/// Runs validation, range filtering, ROI cropping, downsampling, and ground removal.
pub fn preprocess_point_cloud(
    mut frame: PointCloudFrame,
    config: &PreprocessingConfig,
) -> io::Result<PointCloudFrame> {
    validate_config(config)?;

    filter_invalid_and_range(&mut frame, config.min_distance_m, config.max_distance_m);

    if let Some(roi) = config.region_of_interest {
        crop_to_region(&mut frame, &roi);
    }

    if let Some(voxel_size_m) = config.voxel_size_m {
        voxel_downsample(&mut frame, voxel_size_m);
    }

    // Ground removal is last so later detection never sees known ground points.
    if let Some(ground) = config.ground_removal {
        remove_ground(&mut frame, &ground);
    }

    Ok(frame)
}

/// Creates an InvalidInput error for an unusable preprocessing configuration.
fn invalid_input(message: impl Into<String>) -> io::Error {
    io::Error::new(ErrorKind::InvalidInput, message.into())
}

/// Validates all enabled preprocessing stages before modifying the frame.
fn validate_config(config: &PreprocessingConfig) -> io::Result<()> {
    if !config.min_distance_m.is_finite()
        || !config.max_distance_m.is_finite()
        || config.min_distance_m < 0.0
        || config.max_distance_m < config.min_distance_m
    {
        return Err(invalid_input(
            "point-cloud distance limits must be finite, non-negative, and ordered",
        ));
    }

    if let Some(roi) = config.region_of_interest {
        roi.validate()?;
    }

    if let Some(voxel_size_m) = config.voxel_size_m {
        if !voxel_size_m.is_finite() || voxel_size_m <= 0.0 {
            return Err(invalid_input(
                "voxel size must be a finite value greater than zero",
            ));
        }
    }

    if let Some(ground) = config.ground_removal {
        if !ground.ground_height_m.is_finite()
            || !ground.tolerance_m.is_finite()
            || ground.tolerance_m < 0.0
        {
            return Err(invalid_input(
                "ground height must be finite and tolerance must be non-negative",
            ));
        }
    }

    Ok(())
}

/// Removes non-finite points and measurements outside the radial distance limits.
fn filter_invalid_and_range(frame: &mut PointCloudFrame, min_distance_m: f32, max_distance_m: f32) {
    let minimum_squared = min_distance_m * min_distance_m;
    let maximum_squared = max_distance_m * max_distance_m;

    frame.points.retain(|point| {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return false;
        }

        let distance_squared = point.x * point.x + point.y * point.y + point.z * point.z;
        distance_squared >= minimum_squared && distance_squared <= maximum_squared
    });
}

/// Retains only points inside the configured axis-aligned region.
fn crop_to_region(frame: &mut PointCloudFrame, roi: &AxisAlignedRoi) {
    frame.points.retain(|point| roi.contains(point));
}

/// Keeps the first point that enters each cubic voxel while preserving input order.
fn voxel_downsample(frame: &mut PointCloudFrame, voxel_size_m: f32) {
    let mut occupied_voxels = HashSet::with_capacity(frame.points.len());
    frame.points.retain(|point| {
        let key = (
            (point.x / voxel_size_m).floor() as i32,
            (point.y / voxel_size_m).floor() as i32,
            (point.z / voxel_size_m).floor() as i32,
        );
        occupied_voxels.insert(key)
    });
}

/// Removes points inside the configured horizontal ground band.
fn remove_ground(frame: &mut PointCloudFrame, config: &GroundRemovalConfig) {
    frame
        .points
        .retain(|point| (point.z - config.ground_height_m).abs() > config.tolerance_m);
}

#[cfg(test)]
#[path = "../../unittest/application_test/pointcloud_processing_test.rs"]
mod tests;
