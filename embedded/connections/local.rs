use std::{
    ffi::OsString,
    fs::{self, File},
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
};

use crate::{devices::lidar::RawPacket, models::pointcloud::PointCloudFrame};

fn create_parent_directory(path: &Path) -> io::Result<()> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    Ok(())
}

pub struct RawCaptureWriter {
    writer: BufWriter<File>,
}

impl RawCaptureWriter {
    pub fn create(path: &Path) -> io::Result<Self> {
        create_parent_directory(path)?;
        Ok(Self {
            writer: BufWriter::new(File::create(path)?),
        })
    }

    pub fn write_packet(&mut self, packet: &RawPacket) -> io::Result<()> {
        self.writer.write_all(packet.as_bytes())
    }

    pub fn finish(mut self) -> io::Result<()> {
        self.writer.flush()
    }
}

pub struct PcdWriter {
    output_path: PathBuf,
    temporary_path: PathBuf,
    data: BufWriter<File>,
    point_count: u64,
}

impl PcdWriter {
    pub fn create(output_path: &Path) -> io::Result<Self> {
        create_parent_directory(output_path)?;
        let mut temporary_name = OsString::from(output_path.as_os_str());
        temporary_name.push(".points.tmp");
        let temporary_path = PathBuf::from(temporary_name);
        let data = BufWriter::new(File::create(&temporary_path)?);

        Ok(Self {
            output_path: output_path.to_owned(),
            temporary_path,
            data,
            point_count: 0,
        })
    }

    pub fn write_frame(&mut self, frame: &PointCloudFrame) -> io::Result<()> {
        for point in &frame.points {
            let timestamp_seconds = point.timestamp_ns as f64 / 1_000_000_000.0;
            writeln!(
                self.data,
                "{:.6} {:.6} {:.6} {} {} {} {:.9}",
                point.x,
                point.y,
                point.z,
                point.intensity,
                point.ring,
                point.return_id,
                timestamp_seconds
            )?;
            self.point_count += 1;
        }
        Ok(())
    }

    pub fn finish(mut self) -> io::Result<u64> {
        self.data.flush()?;
        drop(self.data);

        let mut output = BufWriter::new(File::create(&self.output_path)?);
        writeln!(output, "# .PCD v0.7 - Point Cloud Data file format")?;
        writeln!(output, "VERSION 0.7")?;
        writeln!(output, "FIELDS x y z intensity ring return timestamp")?;
        writeln!(output, "SIZE 4 4 4 1 1 1 8")?;
        writeln!(output, "TYPE F F F U U U F")?;
        writeln!(output, "COUNT 1 1 1 1 1 1 1")?;
        writeln!(output, "WIDTH {}", self.point_count)?;
        writeln!(output, "HEIGHT 1")?;
        writeln!(output, "VIEWPOINT 0 0 0 1 0 0 0")?;
        writeln!(output, "POINTS {}", self.point_count)?;
        writeln!(output, "DATA ascii")?;

        let mut temporary_data = File::open(&self.temporary_path)?;
        io::copy(&mut temporary_data, &mut output)?;
        output.flush()?;
        drop(output);
        drop(temporary_data);
        fs::remove_file(&self.temporary_path)?;
        Ok(self.point_count)
    }
}
