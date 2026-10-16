using System;
using System.Threading;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Globalization;
using System.Net.Http;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Interop;
using Microsoft.Win32;

namespace PcdConverter
{

public partial class MainWindow : Window
{
    private int viewerPort = SplattingTools.DefaultViewerPort;
    private int rememberedPort = SplattingTools.DefaultViewerPort;
    private bool initialized;
    private string repository = "";
    private string lastRepository = "";
    private readonly RepositoryHistory history;
    private readonly CommandRunner runner = new CommandRunner();
    private CancellationTokenSource? cancellation;
    private RunningCommand? viewer;
    private Task? operation;
    private bool busy, closing, allowClose, viewerReady;

    public MainWindow(string repository = "", RepositoryHistory? history = null)
    {
        InitializeComponent();
        this.history = history ?? new RepositoryHistory();
        try { rememberedPort = this.history.LoadViewerPort(); }
        catch (Exception ex) { Log("WARNING: Unable to restore Viewer Port. Using 5173. " + ex.Message); }
        ViewerPortBox.Text = rememberedPort.ToString(CultureInfo.InvariantCulture);
        if (!string.IsNullOrWhiteSpace(repository))
        {
            this.repository = RepositoryValidator.Validate(repository);
            RepositoryBox.Text = this.repository;
            Log("Repository: " + this.repository);
        }
        else RestoreRepository();
        if (this.repository.Length == 0)
            Log("Select a Gaussian Splatting repository to enable generation and the viewer. Plain conversion does not need a repository.");
        Log("The last valid repository is remembered in Windows user preferences. No TXT configuration is needed.");
        Log("Closing this application stops its viewer. Browser tabs are left open.");
        initialized = true;
        UpdateButtons();
    }

    private void ViewerPort_Changed(object sender, TextChangedEventArgs e)
    {
        // TextChanged can fire before InitializeComponent has created all controls.
        if (initialized) UpdateButtons();
    }

    private void ViewerPort_LostFocus(object sender, System.Windows.Input.KeyboardFocusChangedEventArgs e) => RememberViewerPort();

    private void RememberViewerPort()
    {
        if (!initialized || !SplattingTools.TryParseViewerPort(ViewerPortBox.Text, out int port) || port == rememberedPort) return;
        try { history.SaveViewerPort(port); rememberedPort = port; }
        catch (Exception ex) { Log("WARNING: Viewer Port could not be remembered. " + ex.Message); }
    }

    private async void Repository_Click(object sender, RoutedEventArgs e)
    {
        if (busy || closing) return;
        using var dialog = new System.Windows.Forms.FolderBrowserDialog
        {
            Description = "Select the root folder of the lidar_surface_splatting repository.",
            SelectedPath = GetRepositoryDialogFolder(),
            ShowNewFolderButton = false
        };
        if (dialog.ShowDialog(new FolderDialogOwner(new WindowInteropHelper(this).Handle)) != System.Windows.Forms.DialogResult.OK) return;
        await ExecuteAsync(_ => SelectRepositoryAsync(dialog.SelectedPath));
    }

    public async Task SelectRepositoryAsync(string path)
    {
        if (closing) throw new InvalidOperationException("The application is closing.");
        // Validate before stopping a working viewer or replacing the current path.
        string selected = RepositoryValidator.Validate(path);
        if (string.Equals(repository, selected, StringComparison.OrdinalIgnoreCase))
        {
            RememberRepository(selected);
            return;
        }
        await StopViewerAsync();
        repository = selected;
        RepositoryBox.Text = selected;
        RememberRepository(selected);
        Log("Selected repository: " + selected);
        Log("Run Setup / Repair Environment if this repository has not been prepared yet.");
        UpdateButtons();
    }

    private void RestoreRepository()
    {
        try
        {
            lastRepository = history.Load();
            if (lastRepository.Length == 0) return;
            repository = RepositoryValidator.Validate(lastRepository);
            RepositoryBox.Text = repository;
            Log("Restored last repository: " + repository);
        }
        catch (Exception ex)
        {
            repository = "";
            StatusText.Text = "The saved repository is unavailable. Select Repository... to choose another folder.";
            Log("WARNING: Unable to restore the last repository. " + ex.Message);
        }
    }

    private void RememberRepository(string selected)
    {
        lastRepository = selected;
        try { history.Save(selected); }
        catch (Exception ex)
        {
            // A denied/unavailable preference store must not prevent conversion.
            Log("WARNING: Repository selected, but its location could not be remembered. " + ex.Message);
        }
    }

    private string GetRepositoryDialogFolder()
    {
        string candidate = repository.Length > 0 ? repository : lastRepository;
        try
        {
            while (candidate.Length > 0)
            {
                if (Directory.Exists(candidate)) return candidate;
                candidate = Path.GetDirectoryName(candidate) ?? "";
            }
        }
        catch (Exception) { }
        return "";
    }

    private bool RequireRepository()
    {
        try { RepositoryValidator.Validate(repository); return true; }
        catch (Exception ex) { ShowError(ex.Message); return false; }
    }

    private string OutputExtension => ((ComboBoxItem)FormatBox.SelectedItem).Content.ToString();

    private void Format_Changed(object sender, SelectionChangedEventArgs e) => UpdateOutputPreview();

    private void UpdateOutputPreview()
    {
        // SelectionChanged can fire while InitializeComponent is still building controls.
        if (InputBox != null && OutputText != null && FormatBox.SelectedItem != null && InputBox.Text.Length > 0)
            OutputText.Text = "Output: " + Path.ChangeExtension(InputBox.Text, OutputExtension);
    }

    private void Browse_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new OpenFileDialog { Filter = "Point Cloud Data (*.pcd)|*.pcd", CheckFileExists = true };
        if (dialog.ShowDialog(this) != true) return;
        InputBox.Text = dialog.FileName;
        UpdateOutputPreview();
    }

    private async void Check_Click(object sender, RoutedEventArgs e)
    {
        if (!RequireRepository()) return;
        await ExecuteAsync(async token =>
        {
            await SplattingTools.Find().CheckAsync(runner, repository, Log, token);
            Log("Environment check passed.");
        });
    }

    private async void Setup_Click(object sender, RoutedEventArgs e)
    {
        if (!RequireRepository()) return;
        if (MessageBox.Show(this, "This runs uv sync and npm ci in the selected repository.\n\n" +
            "It downloads dependencies and replaces that repository's node_modules directory.\n" +
            "Only run commands in a repository you trust. Continue?", "Prepare environment",
            MessageBoxButton.YesNo, MessageBoxImage.Question) != MessageBoxResult.Yes) return;
        await ExecuteAsync(async token =>
        {
            await StopViewerAsync();
            await SplattingTools.Find().SetupAsync(runner, repository, Log, token);
        });
    }

    private async void Convert_Click(object sender, RoutedEventArgs e) => await ConvertSelectedAsync(false);
    private async void Generate_Click(object sender, RoutedEventArgs e) => await ConvertSelectedAsync(true);

    private async Task ConvertSelectedAsync(bool generate)
    {
        if (generate && !RequireRepository()) return;
        if (generate && !SplattingTools.TryParseViewerPort(ViewerPortBox.Text, out _))
        { ShowError("Viewer Port must be a whole number from 1024 to 65535."); return; }
        string input = InputBox.Text;
        if (!File.Exists(input)) { ShowError("Select an existing PCD file first."); return; }
        string output = Path.ChangeExtension(input, OutputExtension);
        bool overwrite = File.Exists(output);
        if (overwrite && MessageBox.Show(this, $"Replace the existing file?\n\n{output}", "Confirm overwrite",
            MessageBoxButton.YesNo, MessageBoxImage.Question) != MessageBoxResult.Yes) return;
        string gaussianOutput = generate ? Path.Combine(repository, "out", Path.GetFileNameWithoutExtension(input) + "-optimized.ply") : "";
        if (generate && MessageBox.Show(this,
            "This will generate Gaussian files and replace public viewer assets in the selected repository.\n\n" +
            $"Output: {gaussianOutput}\n\nExisting generated files with the same name will be replaced. Continue?",
            "Generate Gaussian files", MessageBoxButton.YesNo, MessageBoxImage.Question) != MessageBoxResult.Yes) return;

        await ExecuteAsync(async token =>
        {
            SplattingTools? tools = null;
            if (generate)
            {
                tools = SplattingTools.Find();
                await tools.CheckAsync(runner, repository, Log, token);
                if (!File.Exists(Path.Combine(repository, ".venv", "Scripts", "python.exe")))
                    throw new IOException("The Python environment is missing. Run Setup / Repair Environment first.");
                await StopViewerAsync(); // Do not serve half-written viewer assets.
            }
            Log("Converting: " + input);
            var progress = new Progress<long>(count => StatusText.Text = $"Converted {count:N0} points...");
            long written = await Task.Run(() => PcdTextConverter.Convert(input, output, overwrite, token, progress), token);
            Log($"Saved {written:N0} points: {output}");
            if (generate)
            {
                Directory.CreateDirectory(Path.GetDirectoryName(gaussianOutput)!);
                await tools!.GenerateAsync(runner, repository, output, gaussianOutput, Log, token);
                Log("Gaussian files saved under: " + Path.GetDirectoryName(gaussianOutput));
                await StartViewerAsync(tools, token);
            }
        });
    }

    private async void Start_Click(object sender, RoutedEventArgs e)
    {
        if (!RequireRepository()) return;
        await ExecuteAsync(token => StartViewerAsync(SplattingTools.Find(), token));
    }

    private async Task StartViewerAsync(SplattingTools tools, CancellationToken token)
    {
        if (viewer?.IsRunning == true) { Log("Viewer is already running."); return; }
        await StopViewerAsync();
        token.ThrowIfCancellationRequested();
        viewerPort = SplattingTools.ParseViewerPort(ViewerPortBox.Text);
        RememberViewerPort();
        viewer = tools.StartViewer(runner, repository, viewerPort, Log);
        var current = viewer;
        viewerReady = false;
        UpdateButtons();
        ViewerText.Text = "Viewer: starting";
        using var http = new HttpClient { Timeout = TimeSpan.FromSeconds(1) };
        try
        {
            bool ready = false;
            for (int attempt = 0; attempt < 80; attempt++)
            {
                token.ThrowIfCancellationRequested();
                if (!current.IsRunning)
                    throw new IOException($"Viewer exited (exit code {await current.Completion}). See Execution Log.");
                try
                {
                    using var response = await http.GetAsync(SplattingTools.ViewerUrl(viewerPort), token);
                    if (response.IsSuccessStatusCode) { ready = true; break; }
                }
                catch (HttpRequestException) { }
                catch (TaskCanceledException) when (!token.IsCancellationRequested) { }
                await Task.Delay(250, token);
            }
            if (!ready) throw new IOException("Viewer did not become ready. See Execution Log.");
            viewerReady = true;
            ViewerText.Text = "Viewer: " + SplattingTools.ViewerUrl(viewerPort);
            UpdateButtons();
            Log("Viewer is ready. Click Open Viewer to open your browser.");
            _ = WatchViewerAsync(current);
        }
        catch { await StopViewerAsync(); throw; }
    }

    private async Task WatchViewerAsync(RunningCommand current)
    {
        try
        {
            int code = await current.Completion;
            if (!closing && ReferenceEquals(viewer, current))
            {
                viewerReady = false;
                ViewerText.Text = "Viewer: stopped";
                Log($"Viewer stopped (exit code {code}).");
                UpdateButtons();
            }
        }
        catch (Exception ex) { if (!closing) Log("Viewer error: " + ex.Message); }
    }

    private async void Stop_Click(object sender, RoutedEventArgs e) => await ExecuteAsync(_ => StopViewerAsync());

    private async Task StopViewerAsync()
    {
        var current = viewer;
        viewer = null;
        viewerReady = false;
        if (current != null) { await runner.StopAsync(current); Log("Viewer and its child processes stopped."); }
        if (!closing) { ViewerText.Text = "Viewer: stopped"; UpdateButtons(); }
    }

    private void Open_Click(object sender, RoutedEventArgs e)
    {
        if (!viewerReady || viewer?.IsRunning != true) return;
        try { Process.Start(new ProcessStartInfo(SplattingTools.ViewerUrl(viewerPort)) { UseShellExecute = true }); }
        catch (Exception ex) { ShowError(ex.Message); }
    }

    private void Cancel_Click(object sender, RoutedEventArgs e) => cancellation?.Cancel();

    private async Task ExecuteAsync(Func<CancellationToken, Task> action)
    {
        if (busy || closing) return;
        busy = true;
        cancellation = new CancellationTokenSource();
        StatusText.Text = "Working...";
        ProgressBar.Visibility = Visibility.Visible;
        ProgressBar.IsIndeterminate = true;
        UpdateButtons();
        try
        {
            operation = action(cancellation.Token);
            await operation;
            StatusText.Text = "Completed.";
        }
        catch (OperationCanceledException) { StatusText.Text = "Cancelled."; Log("Task cancelled."); }
        catch (Exception ex) { Log("ERROR: " + ex.Message); if (!closing) ShowError(ex.Message); }
        finally
        {
            busy = false;
            cancellation.Dispose();
            cancellation = null;
            ProgressBar.IsIndeterminate = false;
            ProgressBar.Visibility = Visibility.Collapsed;
            UpdateButtons();
        }
    }

    private void UpdateButtons()
    {
        foreach (var button in new[] { RepositoryButton, BrowseButton, ConvertButton })
            button.IsEnabled = !busy && !closing;
        foreach (var button in new[] { CheckButton, SetupButton, GenerateButton, StartButton })
            button.IsEnabled = !busy && !closing && repository.Length > 0;
        FormatBox.IsEnabled = !busy && !closing;
        bool validPort = SplattingTools.TryParseViewerPort(ViewerPortBox.Text, out _);
        ViewerPortBox.IsEnabled = !busy && !closing && viewer?.IsRunning != true;
        ViewerPortHint.Text = validPort ? "Remembered automatically. Stop the viewer to change its port."
            : "Enter a whole number from 1024 to 65535.";
        ViewerPortHint.Foreground = validPort ? System.Windows.Media.Brushes.DimGray : System.Windows.Media.Brushes.Firebrick;
        GenerateButton.IsEnabled &= validPort;
        StartButton.IsEnabled &= validPort;
        StopButton.IsEnabled = !busy && !closing && viewer?.IsRunning == true;
        OpenButton.IsEnabled = !closing && viewerReady && viewer?.IsRunning == true;
        CancelButton.IsEnabled = busy && !closing;
    }

    private void Log(string message)
    {
        if (closing || Dispatcher.HasShutdownStarted) return;
        Dispatcher.BeginInvoke(new Action(() =>
        {
            if (closing) return;
            // Keep the UI bounded during long-running Vite output.
            if (LogBox.Text.Length > 150000) LogBox.Text = LogBox.Text.Substring(LogBox.Text.Length - 100000);
            LogBox.AppendText($"[{DateTime.Now:HH:mm:ss}] {message}\n");
            LogBox.ScrollToEnd();
        }));
    }

    private void ShowError(string message)
    {
        StatusText.Text = "Error. See Execution Log.";
        MessageBox.Show(this, message, "PCD Converter", MessageBoxButton.OK, MessageBoxImage.Error);
    }

    private async void Window_Closing(object? sender, CancelEventArgs e)
    {
        if (allowClose) return;
        e.Cancel = true;
        if (closing) return;
        RememberViewerPort();
        closing = true;
        StatusText.Text = "Stopping tasks and viewer...";
        UpdateButtons();
        cancellation?.Cancel();
        try
        {
            // Wait for cancellation cleanup before disposing the shared command runner.
            if (operation != null) { try { await operation; } catch (Exception) { } }
            await StopViewerAsync();
            await runner.DisposeAsync();
        }
        catch (Exception ex)
        {
            MessageBox.Show(this, "Shutdown cleanup failed: " + ex.Message, "PCD Converter", MessageBoxButton.OK, MessageBoxImage.Warning);
        }
        allowClose = true;
        // Even idle cleanup can finish synchronously inside the first Closing event.
        // Queue the final close so it never recursively closes an already-closing window.
        _ = Dispatcher.BeginInvoke(new Action(Close));
    }

    private sealed class FolderDialogOwner : System.Windows.Forms.IWin32Window
    {
        public IntPtr Handle { get; }
        public FolderDialogOwner(IntPtr handle) { Handle = handle; }
    }
}
}
