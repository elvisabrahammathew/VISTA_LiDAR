//! Validation tests for the dependency-free HTTP transport.

use super::*;

#[test]
fn rejects_invalid_endpoint_values() {
    assert!(HttpClient::new("", 3000, Duration::from_secs(1)).is_err());
    assert!(HttpClient::new("localhost", 0, Duration::from_secs(1)).is_err());
    assert!(HttpClient::new("localhost", 3000, Duration::ZERO).is_err());
}

#[test]
fn parses_http_response() {
    let response = parse_response("HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nok").unwrap();
    assert_eq!(response.status_code, 200);
    assert_eq!(response.reason, "OK");
    assert_eq!(response.body, "ok");
}
