using System;
using System.IO;
using System.Text.RegularExpressions;

namespace RawDataSender
{
    public sealed class ReplayOptions
    {
        public string FilePath { get; set; } = "";
        public string PortName { get; set; } = "";
        public int BaudRate { get; set; } = 4000000;
        public double Speed { get; set; } = 1;
        public bool Loop { get; set; }

        public void Validate()
        {
            if (string.IsNullOrWhiteSpace(FilePath) || !File.Exists(FilePath))
                throw new ArgumentException("Choose an existing BIN file first.");
            if (!Regex.IsMatch(PortName ?? "", @"\ACOM[1-9][0-9]{0,4}\z", RegexOptions.IgnoreCase))
                throw new ArgumentException("Select an output COM port.");
            if (BaudRate < 300 || BaudRate > 12000000)
                throw new ArgumentException("Baud rate must be an integer from 300 to 12000000.");
            if (double.IsNaN(Speed) || double.IsInfinity(Speed) || Speed < 0.1 || Speed > 4)
                throw new ArgumentException("Send rate must be from 0.1x to 4x.");
        }
    }

    public sealed class ReplayResult
    {
        public long FileBytes { get; internal set; }
        public long BytesSent { get; internal set; }
        public long Passes { get; internal set; }
        public double MaximumLagSeconds { get; internal set; }
    }

    public sealed class PlaybackProgress
    {
        public string Phase { get; set; } = "";
        public long Position { get; set; }
        public long Length { get; set; }
        public long TotalBytes { get; set; }
        public long Pass { get; set; }
        public double ElapsedSeconds { get; set; }
        public double MaximumLagSeconds { get; set; }
        public double Fraction => Length == 0 ? 0 : Math.Min(1, (double)Position / Length);
    }
}
