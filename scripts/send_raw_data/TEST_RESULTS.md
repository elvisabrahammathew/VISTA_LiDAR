# Binary-only sender verification — 19 October 2026 (Australia/Sydney)

This supersedes the earlier timed Unitree replay implementation and its 44-test
report. The protocol parser was removed; the application now only sends bytes.

## Builds and automated tests

- Visual Studio 2019 Community MSBuild, .NET Framework 4.7.2, Any CPU.
- Release: 31 passed, 0 failed.
- Debug: 31 passed, 0 failed.
- Tests are built directly from Tests/RawDataSender.Tests.csproj, including the
  matching WPF project reference. The existing solution maps its test project's
  Debug configuration to Release; that user solution mapping was preserved.
  An initial run of the old Debug test executable was not counted as validation.
- The publish EXE matches bin/Release/RawDataSender.exe by SHA256:
  CC990D750AD94A5EB6F5EFC693760B7CFE384265745701EFFFCE0B5672C09895.
- The reduced WPF layout was rendered and visually inspected. It has no mode
  selector, content-validation button, protocol explanation or COM warning banner.

Tests cover arbitrary bytes, malformed/incomplete protocol-looking contents,
unchanged payloads across loops, ignored timestamp-like contents, bounded blocks,
byte-count pacing including final partial blocks, send rate, read-only source
access, empty/missing file errors, cancellation, failures/disposal, queue stalls,
preferences and WPF Start/Stop/Close behavior.

## Read-only real-file byte-equality checks

The production binary sender was exercised using a simulated clock and hash-only
output, with no serial port opened. Both output hashes exactly match their source.

| File | Bytes sent | Simulated nominal send duration at 4000000 baud / 1x |
|---|---:|---:|
| 20261016_133012.bin | 283,687,612 | 709.219030 s |
| 20261018_203122.bin | 61,211,764 | 153.029410 s |

Source SHA256 = sender output SHA256:

    20261016_133012.bin
    BDE53B2D243CC8E9A2D553E618DDF900058E0E14202B0166BA0C3CB7FB9C6734

    20261018_203122.bin
    B83B935A8E28A57AA6943E0A720C16869C568F9A6319C1436023E1A0E9ECA67B

These durations are bytes * 10 / baud, NOT device-time spans or measured UART
delivery times. SHA256 exists in the test helper only; the application does not
scan the input for checksums, headers, frames or timestamps.

## Scope and limitations

No actual/virtual COM transmission, driver latency test, C++ receiver integration,
localization accuracy test or physical LiDAR operation was performed. No C++,
RAW, receiver config, Grafana, COM drivers or unrelated project files were changed.
The WPF application is Windows-only; Linux/ARM C++ builds do not apply to this edit.

Existing file/directory/baud preferences are retained. UnitreeRaw.cs and its
project reference were removed intentionally as part of the binary-only request.
No user data was deleted. The publish folder remains trackable; nothing was
staged or committed.
