use std::{io, net::TcpStream, time::Duration};

pub const PLATFORM_NAME: &str = "Linux";

pub fn configure_sensor_stream(stream: &TcpStream) -> io::Result<()> {
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(Duration::from_secs(5)))
}
