using System;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Threading;

namespace RawDataSender
{
    public interface IByteOutput : IDisposable
    {
        void Write(byte[] bytes, int count);
        int PendingBytes { get; }
    }

    public sealed class SerialOutput : IByteOutput
    {
        private readonly SerialPort port;
        public SerialOutput(ReplayOptions options)
        {
            port = new SerialPort(options.PortName, options.BaudRate, Parity.None, 8, StopBits.One)
            {
                Handshake = Handshake.None,
                DtrEnable = true, RtsEnable = true,
                WriteTimeout = 1000, ReadTimeout = 1000, WriteBufferSize = 16384
            };
            try { port.Open(); }
            catch { port.Dispose(); throw; }
        }
        public void Write(byte[] bytes, int count) => port.Write(bytes, 0, count);
        public int PendingBytes => port.BytesToWrite;
        public void Dispose()
        {
            // After cancellation, do not deliberately flush a queued replay tail.
            // On normal completion ReplayService has already awaited an empty queue.
            try { if (port.IsOpen) port.DiscardOutBuffer(); }
            catch (IOException) { } // Device unplugged while stopping.
            catch (InvalidOperationException) { }
            finally { port.Dispose(); }
        }
    }

    public interface IReplayClock
    {
        double Seconds { get; }
        void WaitUntil(double deadlineSeconds, CancellationToken token);
    }

    public sealed class ReplayClock : IReplayClock
    {
        private readonly Stopwatch watch = Stopwatch.StartNew();
        public double Seconds => watch.Elapsed.TotalSeconds;
        public void WaitUntil(double deadlineSeconds, CancellationToken token)
        {
            while (true)
            {
                token.ThrowIfCancellationRequested();
                double remaining = deadlineSeconds - Seconds;
                if (remaining <= 0) return;
                // Absolute deadlines avoid cumulative sleep/write overhead. No busy spin.
                int milliseconds = Math.Max(1, (int)Math.Min(50, remaining * 1000));
                if (token.WaitHandle.WaitOne(milliseconds)) token.ThrowIfCancellationRequested();
            }
        }
    }

    // Balanced, scoped Windows timer request. Actual scheduling is still best-effort.
    internal sealed class TimerResolution : IDisposable
    {
        [DllImport("winmm.dll")] private static extern uint timeBeginPeriod(uint period);
        [DllImport("winmm.dll")] private static extern uint timeEndPeriod(uint period);
        private readonly bool active;
        public TimerResolution() { active = timeBeginPeriod(1) == 0; }
        public void Dispose() { if (active) timeEndPeriod(1); }
    }
}
