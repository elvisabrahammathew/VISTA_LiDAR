using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using Microsoft.Win32;
using RawDataSender;

internal static class Program
{
    private static int passed, failed;
    private static string root = "";
    private static readonly List<string> keys = new List<string>();

    [STAThread]
    private static int Main(string[] args)
    {
        if (args.Length == 2 && args[0] == "--replay-check") return CheckReal(args[1]);
        if (args.Length == 2 && args[0] == "--render") { Render(args[1]); return 0; }
        root = Path.Combine(Path.GetTempPath(), "RawDataSenderTests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(root);
        SynchronizationContext.SetSynchronizationContext(new DispatcherSynchronizationContext(Dispatcher.CurrentDispatcher));
        Console.WriteLine("Fixtures: " + root);

        Test("All byte values are sent unchanged", () =>
        {
            byte[] bytes = Enumerable.Range(0, 256).Select(i => (byte)i).ToArray();
            var output = new Output();
            var result = Service(output, new Clock()).Run(Options(bytes), CancellationToken.None);
            Bytes(bytes, output.Data.ToArray()); Equal(256L, result.BytesSent);
            Equal(256L, result.FileBytes); Equal(1L, result.Passes); True(output.Disposed);
        });
        Test("Incomplete headers, invalid checksums and arbitrary trailing bytes are sent unchanged", () =>
        {
            byte[] bytes = { 0x55, 0xaa, 5, 10, 104, 0, 0, 0, 255, 255, 255, 255, 99, 0, 255, 0x55 };
            var output = new Output(); Service(output, new Clock()).Run(Options(bytes), CancellationToken.None);
            Bytes(bytes, output.Data.ToArray());
        });
        Test("Timestamp-like contents do not affect transmission", () =>
        {
            byte[] bytes = Enumerable.Repeat((byte)255, 200).ToArray();
            var clock = new Clock(); Service(new Output(), clock).Run(Options(bytes), CancellationToken.None);
            Near(200 * 10.0 / 4000000, clock.Seconds);
        });
        Test("Large files stream in blocks no larger than 4096 bytes", () =>
        {
            byte[] bytes = Enumerable.Range(0, 1024 * 1024 + 17).Select(i => (byte)(i % 251)).ToArray();
            var output = new Output(); Service(output, new Clock()).Run(Options(bytes), CancellationToken.None);
            True(output.MaximumWrite <= 4096); Bytes(bytes, output.Data.ToArray());
        });
        Test("Low baud uses bounded small blocks", () =>
        {
            var options = Options(new byte[100]); options.BaudRate = 300;
            var output = new Output(); Service(output, new Clock()).Run(options, CancellationToken.None);
            True(output.MaximumWrite <= 6); Equal(100L, output.Data.Length);
        });
        Test("Absolute byte-count pacing includes the last partial block", () =>
        {
            var clock = new Clock();
            Service(new Output(), clock).Run(Options(new byte[5003]), CancellationToken.None);
            Near(0, clock.Deadlines[0]); Near(4096 * 10.0 / 4000000, clock.Deadlines[1]);
            Near(5003 * 10.0 / 4000000, clock.Seconds);
        });
        Test("A single-byte file is paced too", () =>
        {
            var clock = new Clock(); Service(new Output(), clock).Run(Options(new byte[] { 42 }), CancellationToken.None);
            Near(10.0 / 4000000, clock.Seconds);
        });
        Test("Send rate changes pacing but never bytes", () =>
        {
            var options = Options(new byte[5003]); options.Speed = 2;
            var clock = new Clock(); var output = new Output();
            Service(output, clock).Run(options, CancellationToken.None);
            Near(5003 * 10.0 / 4000000 / 2, clock.Seconds); Bytes(new byte[5003], output.Data.ToArray());
        });
        Test("Loop repeats the complete file byte-for-byte without changing any payload", () =>
        {
            byte[] bytes = Enumerable.Range(0, 5003).Select(i => (byte)(i % 251)).ToArray();
            var options = Options(bytes); options.Loop = true;
            using var stop = new CancellationTokenSource();
            var output = new Output { OnWrite = n => { if (n == 4) stop.Cancel(); } };
            Throws<OperationCanceledException>(() => Service(output, new Clock()).Run(options, stop.Token));
            Bytes(bytes.Concat(bytes).ToArray(), output.Data.ToArray()); True(output.Disposed);
            Bytes(bytes, File.ReadAllBytes(options.FilePath));
        });
        Test("Empty file does not open COM", () =>
        {
            int opens = 0;
            Throws<InvalidDataException>(() => new ReplayService(o => { opens++; return new Output(); })
                .Run(Options(new byte[0]), CancellationToken.None));
            Equal(0, opens);
        });
        Test("Missing file does not open COM", () =>
        {
            int opens = 0; var options = Options(new byte[1]); options.FilePath += ".missing";
            Throws<ArgumentException>(() => new ReplayService(o => { opens++; return new Output(); }).Run(options, CancellationToken.None));
            Equal(0, opens);
        });
        Test("Cancellation before start does not open COM", () =>
        {
            using var stop = new CancellationTokenSource(); stop.Cancel(); int opens = 0;
            Throws<OperationCanceledException>(() => new ReplayService(o => { opens++; return new Output(); })
                .Run(Options(new byte[1]), stop.Token)); Equal(0, opens);
        });
        Test("Cancellation while waiting releases output", () =>
        {
            using var stop = new CancellationTokenSource();
            var output = new Output(); var clock = new Clock { BeforeWait = () => stop.Cancel() };
            Throws<OperationCanceledException>(() => Service(output, clock).Run(Options(new byte[1]), stop.Token));
            True(output.Disposed); Equal(0L, output.Data.Length);
        });
        Test("Write errors release output without retrying bytes", () =>
        {
            var output = new Output { WriteError = new IOException("disconnected") };
            Throws<IOException>(() => Service(output, new Clock()).Run(Options(new byte[1]), CancellationToken.None));
            Equal(1, output.Writes); True(output.Disposed);
        });
        Test("Cancellation wins over a write timeout", () =>
        {
            using var stop = new CancellationTokenSource();
            var output = new Output { OnWrite = n => stop.Cancel(), WriteError = new TimeoutException("write") };
            Throws<OperationCanceledException>(() => Service(output, new Clock()).Run(Options(new byte[1]), stop.Token));
            True(output.Disposed);
        });
        Test("Stalled COM queue times out and releases output", () =>
        {
            var output = new Output { Pending = 100 };
            Throws<TimeoutException>(() => Service(output, new Clock()).Run(Options(new byte[1]), CancellationToken.None));
            True(output.Disposed);
        });
        Test("Pending COM queue drains before completion", () =>
        {
            int reads = 0; var output = new Output { ReadPending = () => ++reads < 4 ? 100 : 0 };
            var clock = new Clock(); Service(output, clock).Run(Options(new byte[1]), CancellationToken.None);
            True(clock.Seconds >= 0.03); True(output.Disposed);
        });
        Test("Progress is throttled and completes at pass one", () =>
        {
            var progress = new Reports();
            Service(new Output(), new Clock()).Run(Options(new byte[800000]), CancellationToken.None, progress);
            True(progress.Items.Count < 20);
            var last = progress.Items.Last(); Equal("Completed", last.Phase); Equal(1L, last.Pass);
            Near(1, last.Fraction); Equal(800000L, last.TotalBytes);
        });
        Test("Replay source is read-only and concurrent writes are denied", () =>
        {
            byte[] bytes = { 1, 2, 3 }; var options = Options(bytes);
            var output = new Output { OnWrite = n => Throws<IOException>(() => { using var file = File.OpenWrite(options.FilePath); }) };
            Service(output, new Clock()).Run(options, CancellationToken.None);
            Bytes(bytes, File.ReadAllBytes(options.FilePath));
        });
        Test("Invalid COM selections are rejected", () =>
        {
            var options = Options(new byte[1]);
            foreach (string port in new[] { "", "COM0", "COM-1", "COM5;anything", @"\\.\COM5" })
            { options.PortName = port; Throws<ArgumentException>(() => options.Validate()); }
            options.PortName = "COM123"; options.Validate();
        });
        Test("Invalid baud and rate values are rejected", () =>
        {
            var options = Options(new byte[1]);
            foreach (int baud in new[] { 0, 299, 12000001 })
            { options.BaudRate = baud; Throws<ArgumentException>(() => options.Validate()); }
            options.BaudRate = 4000000;
            foreach (double speed in new[] { double.NaN, double.PositiveInfinity, 0, 4.1 })
            { options.Speed = speed; Throws<ArgumentException>(() => options.Validate()); }
        });
        Test("Preferences retain Unicode file, folder, port and baud", () =>
        {
            var pref = Preferences(); string path = Path.Combine(root, "capture Ω % literal.bin");
            File.WriteAllBytes(path, new byte[] { 1 }); pref.SaveFile(path); pref.SaveConnection("COM123", 4000000);
            Equal(path, pref.LoadFile()); Equal(root, pref.LoadDirectory());
            Equal("COM123", pref.LoadPort()); Equal(4000000, pref.LoadBaud());
            Throws<FileNotFoundException>(() => pref.SaveFile(path + ".missing")); Equal(path, pref.LoadFile());
        });
        Test("Missing preferences do not create registry keys", () =>
        {
            var pref = Preferences(); string path = keys.Last();
            Equal("", pref.LoadFile()); Equal("", pref.LoadDirectory()); Equal(4000000, pref.LoadBaud());
            using var key = Registry.CurrentUser.OpenSubKey(path); True(key == null);
        });
        Test("Corrupt preferences use safe defaults", () =>
        {
            var pref = Preferences();
            using (var key = Registry.CurrentUser.CreateSubKey(keys.Last()))
            { key!.SetValue("LastFile", 7); key.SetValue("BaudRate", -1); }
            Equal("", pref.LoadFile()); Equal(4000000, pref.LoadBaud());
        });
        Test("Preference keys cannot escape the application namespace", () =>
            Throws<ArgumentException>(() => new SenderPreferences(@"Software\OtherApplication")));
        Test("WPF has no protocol mode or validation controls", () =>
        {
            var w = new MainWindow(Preferences(), new ReplayService(o => throw new Exception("no COM")));
            True(w.FindName("ModeCombo") == null); True(w.FindName("ValidateButton") == null);
            True(w.FindName("SummaryText") == null); Equal("Start sending", Button(w, "StartButton").Content as string ?? "");
            True(!Button(w, "StopButton").IsEnabled); True(((CheckBox)w.FindName("LoopCheck")).IsChecked == false);
            Equal(2, ((ComboBox)w.FindName("SpeedCombo")).SelectedIndex); w.Close();
        });
        Test("WPF restores the last file and baud", () =>
        {
            var pref = Preferences(); var options = Options(new byte[1]);
            pref.SaveFile(options.FilePath); pref.SaveConnection("COM9999", 921600);
            var w = new MainWindow(pref, new ReplayService(o => throw new Exception("no COM")));
            Equal(options.FilePath, ((TextBox)w.FindName("FileBox")).Text);
            Equal("921600", ((ComboBox)w.FindName("BaudCombo")).Text); w.Close();
        });
        Test("WPF completion restores controls", () =>
        {
            var output = new Output(); var w = new MainWindow(Preferences(), Service(output, new Clock()));
            SetupWindow(w, Options(new byte[] { 1, 2, 3 }).FilePath); Start(w);
            PumpUntil(() => Button(w, "StartButton").IsEnabled);
            Equal("Completed", ((TextBlock)w.FindName("StatusText")).Text); True(output.Disposed); w.Close();
        });
        Test("WPF port-busy error restores controls", () =>
        {
            var w = new MainWindow(Preferences(), new ReplayService(o => throw new UnauthorizedAccessException("Port busy")));
            SetupWindow(w, Options(new byte[1]).FilePath); Start(w);
            PumpUntil(() => Button(w, "StartButton").IsEnabled);
            Equal("Error", ((TextBlock)w.FindName("StatusText")).Text); w.Close();
        });
        Test("WPF Stop cancels sending and releases output", () =>
        {
            using var entered = new ManualResetEventSlim();
            var output = new Output { OnWrite = n => entered.Set() };
            var w = new MainWindow(Preferences(), new ReplayService(o => output));
            SetupWindow(w, Options(new byte[100]).FilePath); ((ComboBox)w.FindName("BaudCombo")).Text = "300"; Start(w);
            PumpUntil(() => entered.IsSet);
            Button(w, "StopButton").RaiseEvent(new RoutedEventArgs(System.Windows.Controls.Button.ClickEvent));
            PumpUntil(() => Button(w, "StartButton").IsEnabled);
            Equal("Stopped", ((TextBlock)w.FindName("StatusText")).Text); True(output.Disposed); w.Close();
        });
        Test("Closing WPF stops sending before the window closes", () =>
        {
            using var entered = new ManualResetEventSlim();
            var output = new Output { OnWrite = n => entered.Set() };
            var w = new MainWindow(Preferences(), new ReplayService(o => output));
            SetupWindow(w, Options(new byte[100]).FilePath); ((ComboBox)w.FindName("BaudCombo")).Text = "300";
            bool closed = false; w.Closed += (s, e) => closed = true; Start(w);
            PumpUntil(() => entered.IsSet); w.Close(); PumpUntil(() => closed); True(output.Disposed);
        });

        Console.WriteLine("RESULT: " + passed + " passed, " + failed + " failed.");
        foreach (string key in keys) Registry.CurrentUser.DeleteSubKeyTree(key, false);
        Console.WriteLine("Fixtures retained: " + root);
        return failed == 0 ? 0 : 1;
    }

    private static int CheckReal(string file)
    {
        var options = new ReplayOptions { FilePath = file, PortName = "COM9999" };
        var output = new HashOutput(); var clock = new Clock { Record = false };
        var result = new ReplayService(o => output, () => clock).Run(options, CancellationToken.None);
        using var original = File.OpenRead(file); using var sha = SHA256.Create();
        string expected = BitConverter.ToString(sha.ComputeHash(original)).Replace("-", "");
        Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "FILE={0}\nBYTES={1}\nSHA256={2}\nOUTPUT_SHA256={3}\nSIMULATED_SEND_SECONDS={4:R}\nCOM_OPENED=False\nOUTPUT_DISPOSED={5}",
            file, result.BytesSent, expected, output.Digest, clock.Seconds, output.Disposed));
        return expected == output.Digest && result.BytesSent == original.Length && output.Disposed ? 0 : 1;
    }

    private static void Render(string path)
    {
        var window = new MainWindow(new SenderPreferences(SenderPreferences.DefaultKeyPath + @"\Tests\Preview"), new ReplayService());
        ((TextBox)window.FindName("FileBox")).Text = @"D:\MASTER\VISTA\LiDar\Repo\data\raw\20261016_133012.bin";
        var ports = (ComboBox)window.FindName("PortCombo"); ports.ItemsSource = new[] { "COM5" }; ports.SelectedIndex = 0;
        var content = (FrameworkElement)window.Content;
        content.Measure(new Size(940, 580)); content.Arrange(new Rect(0, 0, 940, 580)); content.UpdateLayout();
        var bitmap = new RenderTargetBitmap(940, 580, 96, 96, PixelFormats.Pbgra32);
        var background = new DrawingVisual();
        using (var drawing = background.RenderOpen()) drawing.DrawRectangle(window.Background, null, new Rect(0, 0, 940, 580));
        bitmap.Render(background); bitmap.Render(content);
        var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(bitmap));
        using (var file = File.Create(path)) png.Save(file);
        window.Close(); Console.WriteLine("Rendered: " + path);
    }

    private static void SetupWindow(MainWindow w, string path)
    {
        ((TextBox)w.FindName("FileBox")).Text = path;
        var ports = (ComboBox)w.FindName("PortCombo"); ports.ItemsSource = new[] { "COM9999" }; ports.SelectedIndex = 0;
    }
    private static void Start(MainWindow w) => Button(w, "StartButton").RaiseEvent(new RoutedEventArgs(System.Windows.Controls.Button.ClickEvent));
    private static Button Button(MainWindow w, string name) => (Button)w.FindName(name);
    private static void PumpUntil(Func<bool> done)
    {
        var watch = Stopwatch.StartNew();
        while (!done())
        {
            if (watch.Elapsed.TotalSeconds > 5) throw new Exception("UI operation timed out.");
            var frame = new DispatcherFrame();
            Dispatcher.CurrentDispatcher.BeginInvoke(DispatcherPriority.Background, new Action(() => frame.Continue = false));
            Dispatcher.PushFrame(frame); Thread.Sleep(1);
        }
    }
    private static SenderPreferences Preferences()
    {
        string key = SenderPreferences.DefaultKeyPath + @"\Tests\" + Guid.NewGuid().ToString("N"); keys.Add(key); return new SenderPreferences(key);
    }
    private static ReplayService Service(Output output, Clock clock) => new ReplayService(o => output, () => clock);
    private static ReplayOptions Options(byte[] bytes)
    {
        string path = Path.Combine(root, Guid.NewGuid().ToString("N") + ".bin"); File.WriteAllBytes(path, bytes);
        return new ReplayOptions { FilePath = path, PortName = "COM9999" };
    }
    private static void Test(string name, Action test)
    { try { test(); passed++; Console.WriteLine("PASS " + name); } catch (Exception e) { failed++; Console.WriteLine("FAIL " + name + ": " + e); } }
    private static void True(bool value) { if (!value) throw new Exception("Expected true."); }
    private static void Equal<T>(T expected, T actual) { if (!Equals(expected, actual)) throw new Exception("Expected " + expected + "; actual " + actual); }
    private static void Near(double expected, double actual) { if (Math.Abs(expected - actual) > 1e-8) throw new Exception("Expected " + expected + "; actual " + actual); }
    private static void Bytes(byte[] expected, byte[] actual) { True(expected.SequenceEqual(actual)); }
    private static void Throws<T>(Action action) where T : Exception
    { try { action(); } catch (T) { return; } throw new Exception("Expected " + typeof(T).Name); }

    private sealed class Clock : IReplayClock
    {
        public double Seconds { get; private set; }
        public readonly List<double> Deadlines = new List<double>(); public bool Record = true; public Action? BeforeWait;
        public void WaitUntil(double deadline, CancellationToken token)
        { BeforeWait?.Invoke(); token.ThrowIfCancellationRequested(); if (Record) Deadlines.Add(deadline); Seconds = Math.Max(Seconds, deadline); }
    }
    private sealed class Output : IByteOutput
    {
        public readonly MemoryStream Data = new MemoryStream(); public bool Disposed; public int Writes, MaximumWrite, Pending;
        public Func<int>? ReadPending; public Action<int>? OnWrite; public Exception? WriteError;
        public void Write(byte[] b, int count)
        { Writes++; MaximumWrite = Math.Max(count, MaximumWrite); OnWrite?.Invoke(Writes); if (WriteError != null) throw WriteError; Data.Write(b, 0, count); }
        public int PendingBytes => ReadPending?.Invoke() ?? Pending;
        public void Dispose() { Disposed = true; }
    }
    private sealed class HashOutput : IByteOutput
    {
        private readonly SHA256 hash = SHA256.Create(); public string Digest = ""; public bool Disposed;
        public void Write(byte[] b, int count) => hash.TransformBlock(b, 0, count, b, 0);
        public int PendingBytes => 0;
        public void Dispose()
        { hash.TransformFinalBlock(new byte[0], 0, 0); Digest = BitConverter.ToString(hash.Hash!).Replace("-", ""); hash.Dispose(); Disposed = true; }
    }
    private sealed class Reports : IProgress<PlaybackProgress>
    { public readonly List<PlaybackProgress> Items = new List<PlaybackProgress>(); public void Report(PlaybackProgress value) => Items.Add(value); }
}
