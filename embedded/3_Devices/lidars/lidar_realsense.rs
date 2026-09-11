//! Intel RealSense L515 depth driver backed by librealsense 2.50.0.
//!
//! Native calls are intentionally isolated in this file. All other layers use
//! the safe `LidarDevice` interface and the shared `PointCloudFrame` model.

#![allow(unsafe_code)]

#[cfg(not(vista_realsense))]
use std::{io, io::ErrorKind};

#[cfg(not(vista_realsense))]
use crate::{
    devices::lidar::{DepthStreamConfig, LidarDevice, RawPacket},
    models::pointcloud::PointCloudFrame,
};

#[cfg(vista_realsense)]
mod native {
    use std::{
        ffi::{c_char, c_int, c_void, CStr, CString},
        io,
        io::ErrorKind,
        ptr, slice,
    };

    use crate::{
        devices::lidar::{DepthStreamConfig, LidarDevice, RawPacket},
        models::pointcloud::{PointCloudFrame, PointXYZIRT},
    };

    const RS2_API_VERSION: c_int = 25_000;
    const RS2_STREAM_DEPTH: c_int = 1;
    const RS2_FORMAT_Z16: c_int = 1;
    const RS2_CAMERA_INFO_NAME: c_int = 0;
    const RS2_CAMERA_INFO_SERIAL_NUMBER: c_int = 1;
    const BITS_PER_DEPTH_PIXEL: c_int = 16;

    mod ffi {
        use super::{c_char, c_int, c_void};

        #[repr(C)]
        pub struct Rs2Context {
            _private: [u8; 0],
        }

        #[repr(C)]
        pub struct Rs2DeviceList {
            _private: [u8; 0],
        }

        #[repr(C)]
        pub struct Rs2Device {
            _private: [u8; 0],
        }

        #[repr(C)]
        pub struct Rs2Pipeline {
            _private: [u8; 0],
        }

        #[repr(C)]
        pub struct Rs2PipelineProfile {
            _private: [u8; 0],
        }

        #[repr(C)]
        pub struct Rs2Config {
            _private: [u8; 0],
        }

        #[repr(C)]
        pub struct Rs2Frame {
            _private: [u8; 0],
        }

        #[repr(C)]
        pub struct Rs2StreamProfile {
            _private: [u8; 0],
        }

        #[repr(C)]
        pub struct Rs2Error {
            _private: [u8; 0],
        }

        #[repr(C)]
        #[derive(Debug, Clone, Copy, PartialEq)]
        pub struct Rs2Intrinsics {
            pub width: c_int,
            pub height: c_int,
            pub ppx: f32,
            pub ppy: f32,
            pub fx: f32,
            pub fy: f32,
            pub model: c_int,
            pub coeffs: [f32; 5],
        }

        unsafe extern "C" {
            pub fn rs2_get_api_version(error: *mut *mut Rs2Error) -> c_int;
            pub fn rs2_create_context(
                api_version: c_int,
                error: *mut *mut Rs2Error,
            ) -> *mut Rs2Context;
            pub fn rs2_delete_context(context: *mut Rs2Context);
            pub fn rs2_query_devices(
                context: *const Rs2Context,
                error: *mut *mut Rs2Error,
            ) -> *mut Rs2DeviceList;
            pub fn rs2_get_device_count(
                devices: *const Rs2DeviceList,
                error: *mut *mut Rs2Error,
            ) -> c_int;
            pub fn rs2_delete_device_list(devices: *mut Rs2DeviceList);
            pub fn rs2_create_device(
                devices: *const Rs2DeviceList,
                index: c_int,
                error: *mut *mut Rs2Error,
            ) -> *mut Rs2Device;
            pub fn rs2_delete_device(device: *mut Rs2Device);
            pub fn rs2_supports_device_info(
                device: *const Rs2Device,
                info: c_int,
                error: *mut *mut Rs2Error,
            ) -> c_int;
            pub fn rs2_get_device_info(
                device: *const Rs2Device,
                info: c_int,
                error: *mut *mut Rs2Error,
            ) -> *const c_char;
            pub fn rs2_create_pipeline(
                context: *mut Rs2Context,
                error: *mut *mut Rs2Error,
            ) -> *mut Rs2Pipeline;
            pub fn rs2_delete_pipeline(pipeline: *mut Rs2Pipeline);
            pub fn rs2_pipeline_stop(pipeline: *mut Rs2Pipeline, error: *mut *mut Rs2Error);
            pub fn rs2_pipeline_wait_for_frames(
                pipeline: *mut Rs2Pipeline,
                timeout_ms: u32,
                error: *mut *mut Rs2Error,
            ) -> *mut Rs2Frame;
            pub fn rs2_pipeline_start_with_config(
                pipeline: *mut Rs2Pipeline,
                config: *mut Rs2Config,
                error: *mut *mut Rs2Error,
            ) -> *mut Rs2PipelineProfile;
            pub fn rs2_delete_pipeline_profile(profile: *mut Rs2PipelineProfile);
            pub fn rs2_create_config(error: *mut *mut Rs2Error) -> *mut Rs2Config;
            pub fn rs2_delete_config(config: *mut Rs2Config);
            pub fn rs2_config_enable_device(
                config: *mut Rs2Config,
                serial: *const c_char,
                error: *mut *mut Rs2Error,
            );
            pub fn rs2_config_enable_stream(
                config: *mut Rs2Config,
                stream: c_int,
                index: c_int,
                width: c_int,
                height: c_int,
                format: c_int,
                framerate: c_int,
                error: *mut *mut Rs2Error,
            );
            pub fn rs2_embedded_frames_count(
                frameset: *mut Rs2Frame,
                error: *mut *mut Rs2Error,
            ) -> c_int;
            pub fn rs2_extract_frame(
                frameset: *mut Rs2Frame,
                index: c_int,
                error: *mut *mut Rs2Error,
            ) -> *mut Rs2Frame;
            pub fn rs2_release_frame(frame: *mut Rs2Frame);
            pub fn rs2_get_frame_stream_profile(
                frame: *const Rs2Frame,
                error: *mut *mut Rs2Error,
            ) -> *const Rs2StreamProfile;
            pub fn rs2_get_stream_profile_data(
                profile: *const Rs2StreamProfile,
                stream: *mut c_int,
                format: *mut c_int,
                index: *mut c_int,
                unique_id: *mut c_int,
                framerate: *mut c_int,
                error: *mut *mut Rs2Error,
            );
            pub fn rs2_get_video_stream_intrinsics(
                profile: *const Rs2StreamProfile,
                intrinsics: *mut Rs2Intrinsics,
                error: *mut *mut Rs2Error,
            );
            pub fn rs2_get_frame_data_size(
                frame: *const Rs2Frame,
                error: *mut *mut Rs2Error,
            ) -> c_int;
            pub fn rs2_get_frame_data(
                frame: *const Rs2Frame,
                error: *mut *mut Rs2Error,
            ) -> *const c_void;
            pub fn rs2_get_frame_width(frame: *const Rs2Frame, error: *mut *mut Rs2Error) -> c_int;
            pub fn rs2_get_frame_height(frame: *const Rs2Frame, error: *mut *mut Rs2Error)
                -> c_int;
            pub fn rs2_get_frame_stride_in_bytes(
                frame: *const Rs2Frame,
                error: *mut *mut Rs2Error,
            ) -> c_int;
            pub fn rs2_get_frame_bits_per_pixel(
                frame: *const Rs2Frame,
                error: *mut *mut Rs2Error,
            ) -> c_int;
            pub fn rs2_depth_frame_get_units(
                frame: *const Rs2Frame,
                error: *mut *mut Rs2Error,
            ) -> f32;
            pub fn rs2_get_frame_timestamp(
                frame: *const Rs2Frame,
                error: *mut *mut Rs2Error,
            ) -> f64;
            pub fn rs2_deproject_pixel_to_point(
                point: *mut f32,
                intrinsics: *const Rs2Intrinsics,
                pixel: *const f32,
                depth: f32,
            );
            pub fn rs2_get_error_message(error: *const Rs2Error) -> *const c_char;
            pub fn rs2_free_error(error: *mut Rs2Error);
        }
    }

    /// Converts one SDK call and its out-error pointer into an `io::Result`.
    fn sdk_call<T>(operation: impl FnOnce(*mut *mut ffi::Rs2Error) -> T) -> io::Result<T> {
        let mut error = ptr::null_mut();
        let value = operation(&mut error);
        if error.is_null() {
            return Ok(value);
        }

        // SAFETY: librealsense owns the error and guarantees the message stays
        // valid until rs2_free_error is called below.
        let message = unsafe {
            let message_pointer = ffi::rs2_get_error_message(error);
            if message_pointer.is_null() {
                "unknown librealsense error".to_owned()
            } else {
                CStr::from_ptr(message_pointer)
                    .to_string_lossy()
                    .into_owned()
            }
        };
        // SAFETY: `error` was returned by librealsense for this call.
        unsafe { ffi::rs2_free_error(error) };
        Err(io::Error::other(message))
    }

    /// Rejects an unexpected null SDK handle even when no error object was returned.
    fn require_handle<T>(handle: *mut T, name: &str) -> io::Result<*mut T> {
        if handle.is_null() {
            Err(io::Error::other(format!(
                "librealsense returned a null {name}"
            )))
        } else {
            Ok(handle)
        }
    }

    struct ContextHandle(*mut ffi::Rs2Context);

    impl Drop for ContextHandle {
        fn drop(&mut self) {
            // SAFETY: this handle is owned by this guard and deleted once.
            unsafe { ffi::rs2_delete_context(self.0) };
        }
    }

    struct DeviceListHandle(*mut ffi::Rs2DeviceList);

    impl Drop for DeviceListHandle {
        fn drop(&mut self) {
            // SAFETY: this handle is owned by this guard and deleted once.
            unsafe { ffi::rs2_delete_device_list(self.0) };
        }
    }

    struct DeviceHandle(*mut ffi::Rs2Device);

    impl Drop for DeviceHandle {
        fn drop(&mut self) {
            // SAFETY: this handle is owned by this guard and deleted once.
            unsafe { ffi::rs2_delete_device(self.0) };
        }
    }

    struct ConfigHandle(*mut ffi::Rs2Config);

    impl Drop for ConfigHandle {
        fn drop(&mut self) {
            // SAFETY: this handle is owned by this guard and deleted once.
            unsafe { ffi::rs2_delete_config(self.0) };
        }
    }

    struct FrameHandle(*mut ffi::Rs2Frame);

    impl Drop for FrameHandle {
        fn drop(&mut self) {
            // SAFETY: this handle is owned by this guard and released once.
            unsafe { ffi::rs2_release_frame(self.0) };
        }
    }

    struct PipelineHandle {
        pipeline: *mut ffi::Rs2Pipeline,
        profile: *mut ffi::Rs2PipelineProfile,
        started: bool,
    }

    impl Drop for PipelineHandle {
        fn drop(&mut self) {
            if self.started {
                let _ = sdk_call(|error| {
                    // SAFETY: the pipeline is still live and was started once.
                    unsafe { ffi::rs2_pipeline_stop(self.pipeline, error) }
                });
            }
            // SAFETY: both handles are owned by this guard and deleted once.
            unsafe {
                if !self.profile.is_null() {
                    ffi::rs2_delete_pipeline_profile(self.profile);
                }
                ffi::rs2_delete_pipeline(self.pipeline);
            }
        }
    }

    /// Owns an active L515 depth pipeline and cached deprojection rays.
    pub struct RealSenseL515 {
        // Pipeline must be dropped before its context, so field order is intentional.
        pipeline: PipelineHandle,
        _context: ContextHandle,
        frame_timeout_ms: u32,
        width: usize,
        height: usize,
        depth_scale_m: f32,
        intrinsics: Option<ffi::Rs2Intrinsics>,
        rays: Vec<[f32; 3]>,
    }

    impl RealSenseL515 {
        /// Finds an L515, selects its serial number, and starts a Z16 depth stream.
        pub fn connect(config: DepthStreamConfig) -> io::Result<Self> {
            let width = positive_c_int(config.width, "depth width")?;
            let height = positive_c_int(config.height, "depth height")?;
            let frames_per_second = positive_c_int(config.frames_per_second, "depth frame rate")?;
            let frame_timeout_ms = duration_millis_u32(config.frame_timeout)?;

            let runtime_version = runtime_api_version()?;
            if runtime_version != RS2_API_VERSION {
                return Err(io::Error::new(
                    ErrorKind::Unsupported,
                    format!(
                        "librealsense API version {runtime_version} does not match required \
                         version {RS2_API_VERSION} (2.50.0)"
                    ),
                ));
            }

            let context = ContextHandle(require_handle(
                sdk_call(|error| {
                    // SAFETY: the API version and error out-pointer follow the C API contract.
                    unsafe { ffi::rs2_create_context(RS2_API_VERSION, error) }
                })?,
                "context",
            )?);
            let serial = find_l515_serial(context.0)?;
            let serial = CString::new(serial).map_err(|_| {
                io::Error::new(ErrorKind::InvalidData, "L515 serial contains a null byte")
            })?;

            let pipeline_pointer = require_handle(
                sdk_call(|error| {
                    // SAFETY: context is live and the error out-pointer is valid.
                    unsafe { ffi::rs2_create_pipeline(context.0, error) }
                })?,
                "pipeline",
            )?;
            let mut pipeline = PipelineHandle {
                pipeline: pipeline_pointer,
                profile: ptr::null_mut(),
                started: false,
            };

            let stream_config = ConfigHandle(require_handle(
                sdk_call(|error| {
                    // SAFETY: the error out-pointer is valid for the duration of the call.
                    unsafe { ffi::rs2_create_config(error) }
                })?,
                "stream configuration",
            )?);
            sdk_call(|error| {
                // SAFETY: config and serial remain valid throughout the call.
                unsafe { ffi::rs2_config_enable_device(stream_config.0, serial.as_ptr(), error) }
            })?;
            sdk_call(|error| {
                // SAFETY: arguments match rs2_config_enable_stream for a Z16 depth stream.
                unsafe {
                    ffi::rs2_config_enable_stream(
                        stream_config.0,
                        RS2_STREAM_DEPTH,
                        -1,
                        width,
                        height,
                        RS2_FORMAT_Z16,
                        frames_per_second,
                        error,
                    )
                }
            })?;

            pipeline.profile = require_handle(
                sdk_call(|error| {
                    // SAFETY: pipeline and configuration are live and owned locally.
                    unsafe {
                        ffi::rs2_pipeline_start_with_config(
                            pipeline.pipeline,
                            stream_config.0,
                            error,
                        )
                    }
                })?,
                "pipeline profile",
            )?;
            pipeline.started = true;

            Ok(Self {
                pipeline,
                _context: context,
                frame_timeout_ms,
                width: 0,
                height: 0,
                depth_scale_m: 0.0,
                intrinsics: None,
                rays: Vec::new(),
            })
        }

        /// Copies one SDK depth frame into packed little-endian Z16 bytes.
        fn read_depth_frame(&mut self, frame: &FrameHandle) -> io::Result<RawPacket> {
            let width = sdk_call(|error| {
                // SAFETY: frame owns a valid librealsense frame reference.
                unsafe { ffi::rs2_get_frame_width(frame.0, error) }
            })?;
            let height = sdk_call(|error| {
                // SAFETY: frame owns a valid librealsense frame reference.
                unsafe { ffi::rs2_get_frame_height(frame.0, error) }
            })?;
            let stride = sdk_call(|error| {
                // SAFETY: frame owns a valid librealsense frame reference.
                unsafe { ffi::rs2_get_frame_stride_in_bytes(frame.0, error) }
            })?;
            let bits_per_pixel = sdk_call(|error| {
                // SAFETY: frame owns a valid librealsense frame reference.
                unsafe { ffi::rs2_get_frame_bits_per_pixel(frame.0, error) }
            })?;
            if width <= 0 || height <= 0 || stride <= 0 || bits_per_pixel != BITS_PER_DEPTH_PIXEL {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    format!(
                        "unexpected L515 depth frame: {width}x{height}, stride={stride}, \
                         bits-per-pixel={bits_per_pixel}"
                    ),
                ));
            }

            let width = width as usize;
            let height = height as usize;
            let stride = stride as usize;
            let packed_row_size = width
                .checked_mul(2)
                .ok_or_else(|| io::Error::new(ErrorKind::InvalidData, "L515 row size overflow"))?;
            if stride < packed_row_size {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    "L515 frame stride is smaller than its Z16 row",
                ));
            }

            let data_size = sdk_call(|error| {
                // SAFETY: frame owns a valid librealsense frame reference.
                unsafe { ffi::rs2_get_frame_data_size(frame.0, error) }
            })?;
            let required_size = stride.checked_mul(height).ok_or_else(|| {
                io::Error::new(ErrorKind::InvalidData, "L515 frame size overflow")
            })?;
            if data_size < 0 || (data_size as usize) < required_size {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    "L515 frame buffer is smaller than its declared layout",
                ));
            }
            let data_pointer = sdk_call(|error| {
                // SAFETY: frame owns a valid librealsense frame reference.
                unsafe { ffi::rs2_get_frame_data(frame.0, error) }
            })? as *const u8;
            if data_pointer.is_null() {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    "librealsense returned a null depth buffer",
                ));
            }

            let profile = sdk_call(|error| {
                // SAFETY: frame owns a valid librealsense frame reference.
                unsafe { ffi::rs2_get_frame_stream_profile(frame.0, error) }
            })?;
            if profile.is_null() {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    "librealsense returned a null depth stream profile",
                ));
            }
            let mut intrinsics = ffi::Rs2Intrinsics {
                width: 0,
                height: 0,
                ppx: 0.0,
                ppy: 0.0,
                fx: 0.0,
                fy: 0.0,
                model: 0,
                coeffs: [0.0; 5],
            };
            sdk_call(|error| {
                // SAFETY: profile is valid while frame is alive; output points to local storage.
                unsafe { ffi::rs2_get_video_stream_intrinsics(profile, &mut intrinsics, error) }
            })?;
            let depth_scale_m = sdk_call(|error| {
                // SAFETY: frame is a depth frame selected by stream type.
                unsafe { ffi::rs2_depth_frame_get_units(frame.0, error) }
            })?;
            self.update_deprojection(width, height, depth_scale_m, intrinsics)?;

            let timestamp_ms = sdk_call(|error| {
                // SAFETY: frame owns a valid librealsense frame reference.
                unsafe { ffi::rs2_get_frame_timestamp(frame.0, error) }
            })?;
            let timestamp_ns = timestamp_milliseconds_to_ns(timestamp_ms)?;

            // SAFETY: buffer size was checked against the SDK-reported data size,
            // and the frame stays alive while every row is copied.
            let source = unsafe { slice::from_raw_parts(data_pointer, required_size) };
            let mut packed = Vec::with_capacity(packed_row_size * height);
            for row in 0..height {
                let start = row * stride;
                packed.extend_from_slice(&source[start..start + packed_row_size]);
            }

            Ok(RawPacket::new(packed).with_timestamp_ns(timestamp_ns))
        }

        /// Caches a unit-depth ray for each pixel, avoiding one FFI call per point per frame.
        fn update_deprojection(
            &mut self,
            width: usize,
            height: usize,
            depth_scale_m: f32,
            intrinsics: ffi::Rs2Intrinsics,
        ) -> io::Result<()> {
            if !depth_scale_m.is_finite() || depth_scale_m <= 0.0 {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    format!("invalid L515 depth scale: {depth_scale_m}"),
                ));
            }
            if intrinsics.width != width as c_int
                || intrinsics.height != height as c_int
                || !intrinsics.fx.is_finite()
                || !intrinsics.fy.is_finite()
                || intrinsics.fx <= 0.0
                || intrinsics.fy <= 0.0
            {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    "invalid L515 depth intrinsics",
                ));
            }
            // The SDK explicitly cannot deproject this forward-distorted model.
            if intrinsics.model == 1 {
                return Err(io::Error::new(
                    ErrorKind::Unsupported,
                    "modified Brown-Conrady depth deprojection is unsupported",
                ));
            }

            if self.width == width
                && self.height == height
                && self.depth_scale_m == depth_scale_m
                && self.intrinsics == Some(intrinsics)
            {
                return Ok(());
            }

            let point_count = width.checked_mul(height).ok_or_else(|| {
                io::Error::new(ErrorKind::InvalidData, "L515 pixel count overflow")
            })?;
            let mut rays = Vec::with_capacity(point_count);
            for y in 0..height {
                for x in 0..width {
                    let pixel = [x as f32, y as f32];
                    let mut ray = [0.0_f32; 3];
                    // SAFETY: pointers refer to correctly laid-out local arrays and
                    // a C-compatible intrinsics structure for the duration of the call.
                    unsafe {
                        ffi::rs2_deproject_pixel_to_point(
                            ray.as_mut_ptr(),
                            &intrinsics,
                            pixel.as_ptr(),
                            1.0,
                        )
                    };
                    rays.push(ray);
                }
            }

            self.width = width;
            self.height = height;
            self.depth_scale_m = depth_scale_m;
            self.intrinsics = Some(intrinsics);
            self.rays = rays;
            Ok(())
        }
    }

    impl LidarDevice for RealSenseL515 {
        /// Identifies the selected concrete driver through the common facade.
        fn device_name(&self) -> &'static str {
            "Intel RealSense L515"
        }

        /// Waits for one frameset and returns the packed Z16 depth frame as RAW data.
        fn read_raw_packet(&mut self) -> io::Result<RawPacket> {
            let frameset = FrameHandle(require_handle(
                sdk_call(|error| {
                    // SAFETY: the pipeline is live and the timeout is a valid u32.
                    unsafe {
                        ffi::rs2_pipeline_wait_for_frames(
                            self.pipeline.pipeline,
                            self.frame_timeout_ms,
                            error,
                        )
                    }
                })?,
                "frameset",
            )?);
            let frame_count = sdk_call(|error| {
                // SAFETY: frameset owns a valid composite frame reference.
                unsafe { ffi::rs2_embedded_frames_count(frameset.0, error) }
            })?;

            for index in 0..frame_count {
                let frame = FrameHandle(require_handle(
                    sdk_call(|error| {
                        // SAFETY: index is within the SDK-reported frameset count.
                        unsafe { ffi::rs2_extract_frame(frameset.0, index, error) }
                    })?,
                    "embedded frame",
                )?);
                let profile = sdk_call(|error| {
                    // SAFETY: frame owns a valid librealsense frame reference.
                    unsafe { ffi::rs2_get_frame_stream_profile(frame.0, error) }
                })?;
                if profile.is_null() {
                    continue;
                }

                let mut stream = 0;
                let mut format = 0;
                let mut stream_index = 0;
                let mut unique_id = 0;
                let mut frame_rate = 0;
                sdk_call(|error| {
                    // SAFETY: output pointers refer to live local integer values.
                    unsafe {
                        ffi::rs2_get_stream_profile_data(
                            profile,
                            &mut stream,
                            &mut format,
                            &mut stream_index,
                            &mut unique_id,
                            &mut frame_rate,
                            error,
                        )
                    }
                })?;
                if stream == RS2_STREAM_DEPTH && format == RS2_FORMAT_Z16 {
                    return self.read_depth_frame(&frame);
                }
            }

            Err(io::Error::new(
                ErrorKind::InvalidData,
                "L515 frameset does not contain a Z16 depth frame",
            ))
        }

        /// Converts packed Z16 pixels into the shared X-forward/Y-left/Z-up point cloud.
        fn decode_packet(&self, packet: &RawPacket) -> io::Result<PointCloudFrame> {
            if self.rays.is_empty() {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    "L515 calibration is unavailable before the first depth frame",
                ));
            }
            decode_depth_frame(
                packet.as_bytes(),
                &self.rays,
                self.depth_scale_m,
                packet.timestamp_ns().unwrap_or_default(),
            )
        }
    }

    /// Converts the packed Z16 image and cached optical rays into common coordinates.
    pub(super) fn decode_depth_frame(
        bytes: &[u8],
        rays: &[[f32; 3]],
        depth_scale_m: f32,
        timestamp_ns: u64,
    ) -> io::Result<PointCloudFrame> {
        let expected_size = rays.len().checked_mul(2).ok_or_else(|| {
            io::Error::new(ErrorKind::InvalidData, "L515 depth buffer size overflow")
        })?;
        if bytes.len() != expected_size {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "L515 depth buffer contains {} bytes; expected {expected_size}",
                    bytes.len()
                ),
            ));
        }

        let mut points = Vec::with_capacity(rays.len());
        for (pixel, ray) in bytes.chunks_exact(2).zip(rays) {
            let raw_depth = u16::from_le_bytes([pixel[0], pixel[1]]);
            if raw_depth == 0 {
                continue;
            }
            let depth_m = f32::from(raw_depth) * depth_scale_m;

            // RealSense optical coordinates are X-right, Y-down, Z-forward.
            // Convert them to the application convention: X-forward, Y-left, Z-up.
            points.push(PointXYZIRT {
                x: ray[2] * depth_m,
                y: -ray[0] * depth_m,
                z: -ray[1] * depth_m,
                intensity: 0,
                ring: 0,
                return_id: 0,
                timestamp_ns,
            });
        }

        Ok(PointCloudFrame::new(timestamp_ns, points))
    }

    /// Reads the API version directly from the loaded librealsense runtime.
    pub(super) fn runtime_api_version() -> io::Result<c_int> {
        sdk_call(|error| {
            // SAFETY: the error out-pointer is valid for the duration of the call.
            unsafe { ffi::rs2_get_api_version(error) }
        })
    }

    /// Returns the serial number of the first connected device identified as an L515.
    fn find_l515_serial(context: *mut ffi::Rs2Context) -> io::Result<String> {
        let devices = DeviceListHandle(require_handle(
            sdk_call(|error| {
                // SAFETY: context is live and owned by the caller.
                unsafe { ffi::rs2_query_devices(context, error) }
            })?,
            "device list",
        )?);
        let count = sdk_call(|error| {
            // SAFETY: the device list is live for the duration of this call.
            unsafe { ffi::rs2_get_device_count(devices.0, error) }
        })?;
        let mut detected_names = Vec::new();

        for index in 0..count {
            let device = DeviceHandle(require_handle(
                sdk_call(|error| {
                    // SAFETY: index is within the SDK-reported device count.
                    unsafe { ffi::rs2_create_device(devices.0, index, error) }
                })?,
                "device",
            )?);
            let name = device_info(device.0, RS2_CAMERA_INFO_NAME)?
                .unwrap_or_else(|| "Unknown RealSense".to_owned());
            detected_names.push(name.clone());

            if name.to_ascii_uppercase().contains("L515") {
                return device_info(device.0, RS2_CAMERA_INFO_SERIAL_NUMBER)?.ok_or_else(|| {
                    io::Error::new(
                        ErrorKind::InvalidData,
                        "connected L515 does not report a serial number",
                    )
                });
            }
        }

        let details = if detected_names.is_empty() {
            "no RealSense devices were detected".to_owned()
        } else {
            format!("detected: {}", detected_names.join(", "))
        };
        Err(io::Error::new(
            ErrorKind::NotFound,
            format!("Intel RealSense L515 was not found ({details})"),
        ))
    }

    /// Reads one optional UTF-8 camera information field from a device.
    fn device_info(device: *mut ffi::Rs2Device, info: c_int) -> io::Result<Option<String>> {
        let supported = sdk_call(|error| {
            // SAFETY: device is live and info is a valid rs2_camera_info value.
            unsafe { ffi::rs2_supports_device_info(device, info, error) }
        })?;
        if supported == 0 {
            return Ok(None);
        }
        let value = sdk_call(|error| {
            // SAFETY: device is live and info is supported as checked above.
            unsafe { ffi::rs2_get_device_info(device, info, error) }
        })?;
        if value.is_null() {
            return Ok(None);
        }

        // SAFETY: the SDK returns a null-terminated string valid while device is live.
        Ok(Some(
            unsafe { CStr::from_ptr(value) }
                .to_string_lossy()
                .into_owned(),
        ))
    }

    /// Converts a positive u32 configuration value to the C API integer type.
    fn positive_c_int(value: u32, label: &str) -> io::Result<c_int> {
        if value == 0 || value > c_int::MAX as u32 {
            return Err(io::Error::new(
                ErrorKind::InvalidInput,
                format!("{label} must be between 1 and {}", c_int::MAX),
            ));
        }
        Ok(value as c_int)
    }

    /// Converts a positive frame timeout to the millisecond type required by the SDK.
    fn duration_millis_u32(duration: std::time::Duration) -> io::Result<u32> {
        let milliseconds = duration.as_millis();
        if milliseconds == 0 || milliseconds > u128::from(u32::MAX) {
            return Err(io::Error::new(
                ErrorKind::InvalidInput,
                "RealSense frame timeout must be between 1 ms and u32::MAX ms",
            ));
        }
        Ok(milliseconds as u32)
    }

    /// Converts the SDK millisecond timestamp to the shared nanosecond representation.
    fn timestamp_milliseconds_to_ns(timestamp_ms: f64) -> io::Result<u64> {
        let timestamp_ns = timestamp_ms * 1_000_000.0;
        if !timestamp_ns.is_finite() || timestamp_ns < 0.0 || timestamp_ns > u64::MAX as f64 {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("invalid L515 frame timestamp: {timestamp_ms} ms"),
            ));
        }
        Ok(timestamp_ns.round() as u64)
    }
}

#[cfg(vista_realsense)]
pub use native::RealSenseL515;

/// Build-time fallback used when no librealsense binary exists for the target.
#[cfg(not(vista_realsense))]
pub struct RealSenseL515;

#[cfg(not(vista_realsense))]
impl RealSenseL515 {
    /// Reports how to enable the backend instead of silently selecting another device.
    pub fn connect(_config: DepthStreamConfig) -> io::Result<Self> {
        Err(io::Error::new(
            ErrorKind::Unsupported,
            "RealSense backend is unavailable: add the target librealsense library under \
             third_party/realsense and rebuild",
        ))
    }
}

#[cfg(not(vista_realsense))]
impl LidarDevice for RealSenseL515 {
    fn device_name(&self) -> &'static str {
        "Intel RealSense L515 (unavailable)"
    }

    fn read_raw_packet(&mut self) -> io::Result<RawPacket> {
        Err(io::Error::new(
            ErrorKind::Unsupported,
            "RealSense backend is unavailable",
        ))
    }

    fn decode_packet(&self, _packet: &RawPacket) -> io::Result<PointCloudFrame> {
        Err(io::Error::new(
            ErrorKind::Unsupported,
            "RealSense backend is unavailable",
        ))
    }
}

#[cfg(all(test, vista_realsense))]
#[path = "../../unittest/device_test/lidar_realsense_test.rs"]
mod tests;
