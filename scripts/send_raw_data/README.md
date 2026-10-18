# VISTA Raw Data Sender

An English-language Windows WPF application that sends a BIN file, unchanged, to
a COM port. Compatible with Visual Studio 2019 and .NET Framework 4.7.2.

## Run

1. Open publish/RawDataSender.exe and keep RawDataSender.exe.config beside it.
2. Browse to a non-empty BIN file. The last file and folder are remembered.
3. Select the output COM port and baud rate. Refresh ports updates the list.
4. Leave Send rate at 1x, or choose another multiplier.
5. Click Start sending. Stop or closing the window cancels sending and releases
   the application's own COM port.
6. Enable Loop until stopped to repeat the complete file.

The application remembers the selected file, folder, COM port and baud in
HKEY_CURRENT_USER\Software\VISTA\RawDataSender. Loop starts off and Send rate
starts at 1x each time. No separate TXT settings file is required.

## Binary-only behavior

The sender reads the file sequentially in bounded chunks and writes all bytes in
their original order. There is no protocol parser, header/frame recognition,
CRC check, timestamp interpretation, content validation or payload modification.
Arbitrary contents, incomplete packets and trailing bytes are sent as-is.
Only the file's availability/non-empty size and connection settings are checked.

Each loop is an exact repeat of the source bytes. Timestamps, sequence IDs and
checksums inside the file are not changed. The source file itself is read-only
and held against concurrent writers while sending.

Send rate controls nominal byte pacing based on baud and 8N1:
nominal bytes per second = baud / 10 * multiplier.
At 4000000 baud and 1x, the target is 400000 bytes/second. This does not follow
the timestamps in a recording or reconstruct the original acquisition timing.
Physical UART capacity remains limited by the actual baud; a larger multiplier
cannot make the hardware exceed that limit. Windows scheduling and driver
backpressure can also reduce the achieved throughput.

Progress shows the current pass, file percentage, total bytes written, elapsed
time and average throughput. Written bytes do not prove the receiver decoded
them. On normal completion the sender waits for its transmit queue to drain.
Cancellation or a timeout may interrupt the byte stream; already delivered
bytes cannot be retracted.

## Connecting to VISTA for a test

Use an existing virtual COM pair, for example sender COM5 and receiver COM6.
Choose COM5 here and configure the C++ application to read COM6. The sender does
not create virtual ports, install drivers, change DeviceConfig, or stop another
sender. Do not select the physical LiDAR's port as a destination for simulated
sensor output.

Because the file is opaque, recorded timestamps repeat when Loop is enabled.
A timestamp-aware receiver may reject or reinitialize at a loop boundary.
That is intentional: this utility does not alter the file to satisfy a specific
receiver. Restart the receiving session for independent one-pass comparisons.

## Build and publish

Open RawDataSender.sln in Visual Studio 2019 with .NET desktop development and
the .NET Framework 4.7.2 targeting pack installed. Select RawDataSender as the
startup project.

To build, test and refresh the ready-to-run Release distribution:

    powershell -ExecutionPolicy Bypass -File .\build.ps1

The script builds the test project directly, including its reference to the WPF
application, so both use the requested configuration independently of local
solution configuration mappings. Only after successful tests are the EXE and
runtime config copied into publish. The folder is intentionally not ignored by
Git. Nothing is staged or committed automatically.

Requires Windows with .NET Framework 4.7.2 or later. No Python, ROS, NuGet runtime
dependency, or modern .NET SDK is needed to run the published application.

## Tests

The dependency-free test executable uses fake outputs; it never opens a COM
port. Coverage includes arbitrary binary data, byte-identical loops, bounded
streaming and pacing, cancellation/errors, preferences and WPF lifecycle.

For an independent byte-equality check with a real capture:

    .\Tests\bin\Release\RawDataSender.Tests.exe --replay-check 'D:\path\capture.bin'

This diagnostic uses a simulated clock and hash-only output, not a serial port.
SHA256 is used by the test helper to compare bytes, not by the application to
interpret or validate the file. See TEST_RESULTS.md for current results.
