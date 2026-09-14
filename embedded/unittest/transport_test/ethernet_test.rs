//! Unit tests for the cross-platform Ethernet connection.

use std::{
    io::Write,
    net::{Ipv4Addr, TcpListener},
    thread,
    time::Duration,
};

use super::*;

/// Confirms that a device driver can read one complete buffer through Transport.
#[test]
fn connects_and_reads_exact_bytes() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let address = listener.local_addr().unwrap();
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream.write_all(&[0x10, 0x20, 0x30, 0x40]).unwrap();
    });

    let mut connection = EthernetConnection::connect(address, Duration::from_secs(1)).unwrap();
    let mut bytes = [0_u8; 4];
    connection.read_exact(&mut bytes).unwrap();

    assert_eq!(bytes, [0x10, 0x20, 0x30, 0x40]);
    server.join().unwrap();
}
