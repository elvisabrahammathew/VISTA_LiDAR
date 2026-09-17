//! Cross-platform Ethernet connection used by TCP-based sensor drivers.

use std::{
    io::{self, Read},
    net::{SocketAddr, TcpStream},
    time::Duration,
};

use crate::platform;

/// Owns one configured TCP stream without exposing OS details to device drivers.
pub struct EthernetConnection {
    stream: TcpStream,
}

impl EthernetConnection {
    /// Opens a TCP connection and applies the Windows/Linux stream configuration.
    pub fn connect(
        address: SocketAddr,
        connect_timeout: Duration,
        read_timeout: Duration,
    ) -> io::Result<Self> {
        let stream = TcpStream::connect_timeout(&address, connect_timeout).map_err(|error| {
            io::Error::new(
                error.kind(),
                format!("Ethernet connection to {address} failed: {error}"),
            )
        })?;
        platform::configure_tcp_stream(&stream, read_timeout)?;
        Ok(Self { stream })
    }

    /// Reads exactly one device-protocol buffer, including across TCP segments.
    pub fn read_exact(&mut self, buffer: &mut [u8]) -> io::Result<()> {
        self.stream.read_exact(buffer)
    }
}

#[cfg(test)]
#[path = "../../../unittest/transport_test/ethernet_test.rs"]
mod tests;
