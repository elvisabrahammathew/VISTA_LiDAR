using System;
using System.IO;
using System.Threading;

namespace RawDataSender
{
    public sealed class ReplayService
    {
        private readonly Func<ReplayOptions, IByteOutput> openOutput;
        private readonly Func<IReplayClock> makeClock;

        public ReplayService(Func<ReplayOptions, IByteOutput>? openOutput = null, Func<IReplayClock>? makeClock = null)
        {
            this.openOutput = openOutput ?? (o => new SerialOutput(o));
            this.makeClock = makeClock ?? (() => new ReplayClock());
        }

        public ReplayResult Run(ReplayOptions options, CancellationToken token,
            IProgress<PlaybackProgress>? progress = null)
        {
            options.Validate();
            token.ThrowIfCancellationRequested();
            // Read-only streaming: bytes are never parsed, modified, or skipped.
            using var input = new FileStream(options.FilePath, FileMode.Open, FileAccess.Read, FileShare.Read, 65536);
            if (input.Length == 0) throw new InvalidDataException("The BIN file is empty.");
            using var resolution = new TimerResolution();
            using var output = openOutput(options);
            var clock = makeClock();
            var result = new ReplayResult { FileBytes = input.Length };
            double start = clock.Seconds, nextReport = start;
            long pass = 1;
            void Report(bool force, string phase = "Sending")
            {
                double now = clock.Seconds;
                if (!force && now < nextReport) return;
                nextReport = now + 0.2;
                progress?.Report(new PlaybackProgress
                {
                    Phase = phase, Position = input.Position, Length = input.Length,
                    TotalBytes = result.BytesSent, Pass = pass,
                    ElapsedSeconds = now - start, MaximumLagSeconds = result.MaximumLagSeconds
                });
            }

            int blockSize = Math.Max(1, Math.Min(4096, options.BaudRate / 50));
            var buffer = new byte[blockSize];
            double secondsPerByte = 10.0 / options.BaudRate / options.Speed; // 8N1.
            try
            {
                do
                {
                    token.ThrowIfCancellationRequested();
                    input.Position = 0;
                    Report(true);
                    int count;
                    while ((count = input.Read(buffer, 0, buffer.Length)) != 0)
                    {
                        double target = start + result.BytesSent * secondsPerByte;
                        clock.WaitUntil(target, token);
                        token.ThrowIfCancellationRequested();
                        result.MaximumLagSeconds = Math.Max(result.MaximumLagSeconds, clock.Seconds - target);
                        output.Write(buffer, count);
                        result.BytesSent = checked(result.BytesSent + count);
                        Report(false);
                    }
                    // Include the last block in the pacing, even for tiny files and loops.
                    clock.WaitUntil(start + result.BytesSent * secondsPerByte, token);
                    result.Passes = pass;
                    Report(true);
                    if (!options.Loop) break;
                    pass = checked(pass + 1);
                } while (true);

                double lastDrainProgress = clock.Seconds;
                int pending = output.PendingBytes, lastPending = pending;
                double stallLimit = Math.Max(3, pending * 10.0 / options.BaudRate + 2);
                while (pending > 0)
                {
                    token.ThrowIfCancellationRequested();
                    if (pending < lastPending) lastDrainProgress = clock.Seconds;
                    if (clock.Seconds - lastDrainProgress > stallLimit)
                        throw new TimeoutException("The COM transmit queue stopped draining.");
                    lastPending = pending;
                    Report(false, "Draining COM queue");
                    clock.WaitUntil(clock.Seconds + 0.01, token);
                    pending = output.PendingBytes;
                }
                token.ThrowIfCancellationRequested();
                Report(true, "Completed");
                return result;
            }
            catch
            {
                token.ThrowIfCancellationRequested();
                throw;
            }
        }
    }
}
