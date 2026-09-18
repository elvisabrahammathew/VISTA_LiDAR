//! Small cross-platform HTTP/1.1 client used by the Grafana Live bridge.

use std::{
    io::{self, Read, Write},
    net::{Shutdown, TcpStream, ToSocketAddrs},
    time::Duration,
};

#[derive(Debug, Clone, Default)]
pub struct HttpResponse {
    pub status_code: u16,
    pub reason: String,
    pub body: String,
}

#[derive(Debug, Clone)]
pub struct HttpClient {
    host: String,
    port: u16,
    timeout: Duration,
}

impl HttpClient {
    pub fn new(host: impl Into<String>, port: u16, timeout: Duration) -> io::Result<Self> {
        let host = host.into();
        validate_header_value(&host, "HTTP host")?;
        if port == 0 || timeout.is_zero() {
            return Err(invalid_input("HTTP port and timeout must be positive"));
        }
        Ok(Self {
            host,
            port,
            timeout,
        })
    }

    pub fn post(
        &self,
        path: &str,
        content_type: &str,
        body: &str,
        headers: &[(String, String)],
    ) -> io::Result<HttpResponse> {
        if !path.starts_with('/') {
            return Err(invalid_input("HTTP path must begin with '/'"));
        }
        validate_header_value(path, "HTTP path")?;
        validate_header_value(content_type, "HTTP content type")?;
        for (name, value) in headers {
            validate_header_name(name)?;
            validate_header_value(value, "HTTP header value")?;
        }

        let address = (self.host.as_str(), self.port)
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| {
                io::Error::new(io::ErrorKind::NotFound, "HTTP host resolved no address")
            })?;
        let mut stream = TcpStream::connect_timeout(&address, self.timeout)?;
        stream.set_read_timeout(Some(self.timeout))?;
        stream.set_write_timeout(Some(self.timeout))?;

        let mut request = format!(
            "POST {path} HTTP/1.1\r\nHost: {}:{}\r\nUser-Agent: vista-sensor-processing/0.1\r\nContent-Type: {content_type}\r\n",
            self.host, self.port
        );
        for (name, value) in headers {
            request.push_str(name);
            request.push_str(": ");
            request.push_str(value);
            request.push_str("\r\n");
        }
        request.push_str(&format!(
            "Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
            body.len()
        ));
        stream.write_all(request.as_bytes())?;
        let _ = stream.shutdown(Shutdown::Write);

        let mut response = Vec::new();
        stream.take(1024 * 1024 + 1).read_to_end(&mut response)?;
        if response.len() > 1024 * 1024 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "HTTP response exceeded 1 MiB",
            ));
        }
        parse_response(&String::from_utf8_lossy(&response))
    }
}

fn parse_response(response: &str) -> io::Result<HttpResponse> {
    let (headers, body) = response.split_once("\r\n\r\n").ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidData, "HTTP response has no headers")
    })?;
    let status_line = headers
        .lines()
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "HTTP response has no status"))?;
    let mut parts = status_line.splitn(3, ' ');
    let version = parts.next().unwrap_or_default();
    let status_code = parts
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "HTTP status is missing"))?
        .parse::<u16>()
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "invalid HTTP status"))?;
    if !version.starts_with("HTTP/") {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid HTTP status line",
        ));
    }
    Ok(HttpResponse {
        status_code,
        reason: parts.next().unwrap_or_default().trim().to_owned(),
        body: body.to_owned(),
    })
}

fn validate_header_name(name: &str) -> io::Result<()> {
    if name.is_empty()
        || !name
            .bytes()
            .all(|value| value.is_ascii_alphanumeric() || value == b'-')
    {
        return Err(invalid_input(
            "HTTP header name may contain only letters, digits, and '-'",
        ));
    }
    Ok(())
}

fn validate_header_value(value: &str, label: &str) -> io::Result<()> {
    if value.is_empty() || value.contains(['\r', '\n']) {
        return Err(invalid_input(format!(
            "{label} cannot be empty or contain line breaks"
        )));
    }
    Ok(())
}

fn invalid_input(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message.into())
}

#[cfg(test)]
#[path = "../../unittest/transport_test/http_test.rs"]
mod tests;
