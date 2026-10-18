using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using Microsoft.Win32;

namespace RawDataSender
{
    public partial class MainWindow : Window
    {
        private readonly SenderPreferences preferences;
        private readonly ReplayService service;
        private readonly Queue<string> log = new Queue<string>();
        private CancellationTokenSource? cancellation;
        private Task? operation;
        private bool busy, closing, allowClose;
        private string lastDirectory = "";
        private int generation;

        public MainWindow() : this(new SenderPreferences(), new ReplayService()) { }

        public MainWindow(SenderPreferences preferences, ReplayService service)
        {
            InitializeComponent();
            this.preferences = preferences; this.service = service;
            BaudCombo.Text = "4000000";
            try
            {
                FileBox.Text = preferences.LoadFile();
                lastDirectory = preferences.LoadDirectory();
                BaudCombo.Text = preferences.LoadBaud().ToString(CultureInfo.InvariantCulture);
                RefreshPorts(preferences.LoadPort());
            }
            catch (Exception ex) { Append("Preferences / port list: " + ex.Message); }
            Append("Ready.");
        }

        private void Append(string message)
        {
            log.Enqueue("[" + DateTime.Now.ToString("HH:mm:ss", CultureInfo.InvariantCulture) + "] " + message);
            while (log.Count > 200) log.Dequeue();
            LogBox.Text = string.Join(Environment.NewLine, log);
            LogBox.ScrollToEnd();
        }

        private void RefreshPorts(string selected)
        {
            string[] ports = SerialPort.GetPortNames().Distinct(StringComparer.OrdinalIgnoreCase)
                .OrderBy(p => p.Length).ThenBy(p => p, StringComparer.OrdinalIgnoreCase).ToArray();
            PortCombo.ItemsSource = ports;
            PortCombo.SelectedItem = ports.FirstOrDefault(p => string.Equals(p, selected, StringComparison.OrdinalIgnoreCase));
            if (ports.Length == 0) Append("No COM ports found.");
        }

        private void Refresh_Click(object sender, RoutedEventArgs e)
        {
            try { RefreshPorts(PortCombo.SelectedItem as string ?? ""); }
            catch (Exception ex) { Append("ERROR: " + ex.Message); }
        }

        private void Browse_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new OpenFileDialog
            {
                Title = "Select a BIN file", Filter = "BIN files (*.bin)|*.bin|All files (*.*)|*.*", CheckFileExists = true
            };
            string typedDirectory = "";
            try { typedDirectory = Path.GetDirectoryName(FileBox.Text) ?? ""; } catch (ArgumentException) { }
            if (Directory.Exists(typedDirectory)) dialog.InitialDirectory = typedDirectory;
            else if (Directory.Exists(lastDirectory)) dialog.InitialDirectory = lastDirectory;
            if (dialog.ShowDialog(this) != true) return;
            FileBox.Text = dialog.FileName;
            SaveFilePreference(dialog.FileName);
        }

        private void SaveFilePreference(string path)
        {
            try
            {
                lastDirectory = Path.GetDirectoryName(Path.GetFullPath(path)) ?? "";
                preferences.SaveFile(path);
            }
            catch (Exception ex) { Append("Could not save the last folder: " + ex.Message); }
        }

        private ReplayOptions ReadOptions()
        {
            if (!int.TryParse(BaudCombo.Text.Trim(), NumberStyles.None, CultureInfo.InvariantCulture, out int baud))
                throw new ArgumentException("Enter an integer baud rate, for example 4000000.");
            var speedItem = SpeedCombo.SelectedItem as ComboBoxItem;
            var options = new ReplayOptions
            {
                FilePath = FileBox.Text.Trim(), PortName = PortCombo.SelectedItem as string ?? "",
                BaudRate = baud, Speed = double.Parse(speedItem?.Tag as string ?? "1", CultureInfo.InvariantCulture),
                Loop = LoopCheck.IsChecked == true
            };
            options.Validate();
            return options;
        }

        private void SetBusy(bool value)
        {
            busy = value;
            FileBox.IsEnabled = BrowseButton.IsEnabled = SettingsGrid.IsEnabled = StartButton.IsEnabled = !value;
            StopButton.IsEnabled = value;
        }

        private void Start_Click(object sender, RoutedEventArgs e)
        {
            if (busy || closing) return;
            ReplayOptions options;
            try { options = ReadOptions(); }
            catch (Exception ex) { StatusText.Text = "Check settings"; Append("ERROR: " + ex.Message); return; }
            SaveFilePreference(options.FilePath);
            try { preferences.SaveConnection(options.PortName, options.BaudRate); }
            catch (Exception ex) { Append("Could not save connection preferences: " + ex.Message); }
            SetBusy(true);
            FileProgress.Value = 0; ProgressText.Text = "Starting...";
            StatusText.Text = "Opening COM port";
            int run = ++generation;
            cancellation = new CancellationTokenSource();
            Append(string.Format(CultureInfo.InvariantCulture, "Starting: {0} -> {1}, {2} baud, {3:0.##}x, loop={4}",
                options.FilePath, options.PortName, options.BaudRate, options.Speed, options.Loop));
            var progress = new Progress<PlaybackProgress>(p =>
            {
                if (!busy || run != generation) return;
                FileProgress.Value = p.Fraction * 100;
                StatusText.Text = p.Phase;
                ProgressText.Text = string.Format(CultureInfo.InvariantCulture,
                    "Pass {0} | {1:P1} | Sent: {2:N0} bytes | Elapsed: {3:N1} s | Average: {4:N1} KiB/s",
                    p.Pass, p.Fraction, p.TotalBytes, p.ElapsedSeconds,
                    p.ElapsedSeconds > 0 ? p.TotalBytes / 1024.0 / p.ElapsedSeconds : 0);
            });
            operation = ExecuteAsync(options, cancellation.Token, progress);
        }

        private async Task ExecuteAsync(ReplayOptions options, CancellationToken token, IProgress<PlaybackProgress> progress)
        {
            try
            {
                ReplayResult result = await Task.Run(() => service.Run(options, token, progress));
                FileProgress.Value = 100;
                StatusText.Text = "Completed";
                Append(string.Format(CultureInfo.InvariantCulture, "Completed: {0:N0} bytes, {1} pass(es). COM port released.",
                    result.BytesSent, result.Passes));
            }
            catch (OperationCanceledException)
            {
                StatusText.Text = "Stopped"; Append("Stopped. COM port released.");
            }
            catch (Exception ex)
            {
                StatusText.Text = "Error"; Append("ERROR: " + ex.Message);
            }
            finally
            {
                SetBusy(false);
                cancellation?.Dispose(); cancellation = null;
            }
        }

        private void Stop_Click(object sender, RoutedEventArgs e)
        {
            StopButton.IsEnabled = false; StatusText.Text = "Stopping..."; cancellation?.Cancel();
        }

        private async void Window_Closing(object? sender, CancelEventArgs e)
        {
            if (allowClose || !busy) return;
            e.Cancel = true;
            if (closing) return;
            closing = true; StopButton.IsEnabled = false; StatusText.Text = "Stopping...";
            cancellation?.Cancel();
            if (operation != null) await operation;
            allowClose = true; Close();
        }
    }
}
