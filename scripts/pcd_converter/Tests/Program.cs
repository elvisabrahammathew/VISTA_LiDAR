using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using Microsoft.Win32;
using PcdConverter;

internal static class Program
{
    private static int passed, failed;
    private static string root = "";
    private static readonly List<string> historyKeys = new List<string>();
    private const string Header = "# VISTA test\nVERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 1\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n";
    private const string Data = "1.23456789 -2 3e-5 255\n0 0 0 0\n";

    [STAThread]
    private static int Main(string[] args)
    {
        if (args.FirstOrDefault()?.StartsWith("--helper-") == true) return Helper(args);
        root = Path.Combine(Path.GetTempPath(), "PcdConverterTests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(root);
        Console.WriteLine("Test artifacts: " + root);
        Test("Reading missing history does not create a preference key", () =>
        {
            string keyPath = NewHistoryKey();
            var history = new RepositoryHistory(keyPath);
            Equal("", history.Load());
            Equal(5173, history.LoadViewerPort());
            using var key = Registry.CurrentUser.OpenSubKey(keyPath);
            True(key == null);
        });
        Test("History survives new instances and preserves literal Unicode/percent paths", () =>
        {
            string keyPath = NewHistoryKey();
            string repo = FakeRepository(Path.Combine(NewDirectory(), "repo Ω %PATH%"));
            new RepositoryHistory(keyPath).Save(repo);
            Equal(repo, new RepositoryHistory(keyPath).Load());
            True(!File.Exists(Path.Combine(repo, "SplattingConfig.txt")));
        });
        Test("Invalid selections cannot overwrite the saved valid repository", () =>
        {
            var history = NewHistory();
            string repo = FakeRepository(NewDirectory());
            history.Save(repo);
            Throws<InvalidDataException>(() => history.Save(NewDirectory()));
            Equal(repo, history.Load());
        });
        Test("A malformed preference value is ignored safely", () =>
        {
            string keyPath = NewHistoryKey();
            using (var key = Registry.CurrentUser.CreateSubKey(keyPath))
                key.SetValue("LastRepositoryPath", 123, RegistryValueKind.DWord);
            Equal("", new RepositoryHistory(keyPath).Load());
        });
        Test("Viewer port parsing accepts boundaries and rejects non-integer or unsafe values", () =>
        {
            foreach (int port in new[] { 1024, 5173, 5180, 65535 })
                Equal(port, SplattingTools.ParseViewerPort(" " + port.ToString(CultureInfo.InvariantCulture) + " "));
            foreach (string invalid in new[] { "", "abc", "0", "1023", "65536", "-1", "+5173", "5.173", "5e3", "51 73", "5173 & echo x", "99999999999" })
            {
                True(!SplattingTools.TryParseViewerPort(invalid, out _));
                Throws<ArgumentException>(() => SplattingTools.ParseViewerPort(invalid));
            }
            Throws<ArgumentOutOfRangeException>(() => SplattingTools.CheckViewerPort(0));
        });
        Test("Viewer port preference survives reopening and does not overwrite repository history", () =>
        {
            string keyPath = NewHistoryKey();
            var history = new RepositoryHistory(keyPath);
            string repo = FakeRepository(NewDirectory());
            history.Save(repo);
            history.SaveViewerPort(5180);
            Equal(5180, new RepositoryHistory(keyPath).LoadViewerPort());
            Equal(repo, history.Load());
            Throws<ArgumentOutOfRangeException>(() => history.SaveViewerPort(65536));
            Equal(5180, history.LoadViewerPort());
            history.Save(repo);
            Equal(5180, history.LoadViewerPort());
        });
        Test("Malformed saved viewer ports safely fall back to 5173", () =>
        {
            string keyPath = NewHistoryKey();
            var history = new RepositoryHistory(keyPath);
            using var key = Registry.CurrentUser.CreateSubKey(keyPath);
            foreach (int invalid in new[] { -1, 0, 1023, 65536 })
            {
                key.SetValue("ViewerPort", invalid, RegistryValueKind.DWord);
                Equal(5173, history.LoadViewerPort());
            }
            key.SetValue("ViewerPort", "5180", RegistryValueKind.String);
            Equal(5173, history.LoadViewerPort());
        });
        Test("Repository validation accepts paths containing spaces and symbols", () =>
        {
            string repo = FakeRepository(Path.Combine(NewDirectory(), "repo with spaces & symbols"));
            Equal(repo, RepositoryValidator.Validate(repo));
        });
        Test("Repository validation normalizes a quoted absolute path", () =>
        {
            string repo = FakeRepository(Path.Combine(NewDirectory(), "repo"));
            Equal(repo, RepositoryValidator.Validate("  \"" + repo + "\"  "));
        });
        Test("Empty, missing and incomplete repository folders are rejected", () =>
        {
            foreach (string repo in new[] { "", "  ", Path.Combine(NewDirectory(), "missing"), NewDirectory() })
                Throws<InvalidDataException>(() => RepositoryValidator.Validate(repo));
        });
        Test("Every required repository artifact is checked", () =>
        {
            foreach (string file in new[] { "package.json", "package-lock.json", "pyproject.toml", "uv.lock", "python/lidar_splat_preprocess/cli.py" })
            {
                string repo = FakeRepository(NewDirectory());
                File.Delete(Path.Combine(repo, file));
                Throws<InvalidDataException>(() => RepositoryValidator.Validate(repo));
            }
        });
        Test("PCD conversion preserves filename, XYZ text and uint8 intensity", () =>
        {
            string source = Source(Header + Data, "Room Map & sample.pcd");
            string output = Path.ChangeExtension(source, ".txt");
            Equal(2L, PcdTextConverter.Convert(source, output));
            Equal(Data.Replace("\n", "\r\n"), File.ReadAllText(output));
            Equal(Header + Data, File.ReadAllText(source));
        });
        Test("Conversion is independent of the current numeric locale", () =>
        {
            var previous = CultureInfo.CurrentCulture;
            try
            {
                CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("de-DE");
                string source = Source(Header + Data);
                Equal(2L, PcdTextConverter.Convert(source, Path.ChangeExtension(source, ".txt")));
            }
            finally { CultureInfo.CurrentCulture = previous; }
        });
        Test("Reordered fields and multi-value extra fields are handled", () =>
        {
            string pcd = "FIELDS normal intensity z x y\nSIZE 4 1 4 4 4\nTYPE F U F F F\nCOUNT 3 1 1 1 1\nWIDTH 1\nHEIGHT 1\nDATA ascii\n0 0 1 42 3 1 2\n";
            string source = Source(pcd);
            string output = Path.ChangeExtension(source, ".txt");
            Equal(1L, PcdTextConverter.Convert(source, output));
            Equal("1 2 3 42\r\n", File.ReadAllText(output));
        });
        Test("COUNT and POINTS are optional when the other headers are valid", () =>
        {
            string source = Source((Header + Data).Replace("COUNT 1 1 1 1\n", "").Replace("POINTS 2\n", ""));
            Equal(2L, PcdTextConverter.Convert(source, Path.ChangeExtension(source, ".txt")));
        });
        Test("Existing output is never overwritten without permission", () =>
        {
            string source = Source(Header + Data);
            string output = Path.ChangeExtension(source, ".txt");
            File.WriteAllText(output, "existing");
            Throws<IOException>(() => PcdTextConverter.Convert(source, output));
            Equal("existing", File.ReadAllText(output));
            Equal(2L, PcdTextConverter.Convert(source, output, true));
        });
        Test("Malformed input preserves existing output and removes temporary files", () =>
        {
            foreach (string bad in new[]
            {
                (Header + Data).Replace("DATA ascii", "DATA binary"),
                (Header + Data).Replace("DATA ascii", "DATA binary_compressed"),
                (Header + Data).Replace("1.23456789", "nan"),
                (Header + Data).Replace("1.23456789", "Infinity"),
                (Header + Data).Replace("255", "256"),
                (Header + Data).Replace("255", "-1"),
                Header + "1 2 3 4\n", Header + Data + "1 2 3 4\n",
                (Header + Data).Replace("POINTS 2", "POINTS 3"),
                (Header + Data).Replace("FIELDS x y z intensity", "FIELDS x y z reflectance"),
                (Header + Data).Replace("FIELDS x y z intensity", "FIELDS x x z intensity"),
                (Header + Data).Replace("COUNT 1 1 1 1", "COUNT 2 1 1 1"),
                (Header + Data).Replace("1.23456789 -2 3e-5 255", "1 2 3")
            })
            {
                string source = Source(bad);
                string output = Path.ChangeExtension(source, ".txt");
                File.WriteAllText(output, "keep this");
                Throws<InvalidDataException>(() => PcdTextConverter.Convert(source, output, true));
                Equal("keep this", File.ReadAllText(output));
                Equal(0, Directory.GetFiles(Path.GetDirectoryName(source)!, "*.tmp").Length);
            }
        });
        Test("Cancellation leaves no partial TXT", () =>
        {
            string source = Source(Header + Data);
            string output = Path.ChangeExtension(source, ".txt");
            using var cancellation = new CancellationTokenSource();
            cancellation.Cancel();
            Throws<OperationCanceledException>(() => PcdTextConverter.Convert(source, output, false, cancellation.Token));
            True(!File.Exists(output));
        });
        Test("Cancellation after partial progress preserves existing TXT", () =>
        {
            string content = Header.Replace("WIDTH 2", "WIDTH 20000").Replace("POINTS 2", "POINTS 20000") +
                string.Concat(Enumerable.Repeat("1 2 3 4\n", 20000));
            string source = Source(content);
            string output = Path.ChangeExtension(source, ".txt");
            File.WriteAllText(output, "keep this");
            using var cancellation = new CancellationTokenSource();
            Throws<OperationCanceledException>(() => PcdTextConverter.Convert(source, output, true,
                cancellation.Token, new ImmediateProgress(_ => cancellation.Cancel())));
            Equal("keep this", File.ReadAllText(output));
            Equal(0, Directory.GetFiles(Path.GetDirectoryName(source)!, "*.tmp").Length);
        });
        Test("Command arguments are literal, not shell commands", () => RunAsync(async () =>
        {
            using var runner = new CommandRunner();
            string argument = "path with spaces & %PATH% $value ; 'quote' \"double\"";
            var lines = new List<string>();
            await runner.RunAsync(SelfExecutable(), SelfArguments("--helper-echo", argument), root, line => lines.Add(line), default);
            Equal(argument, Encoding.UTF8.GetString(Convert.FromBase64String(lines.Single())));
        }));
        Test("Windows argument quoting preserves empty, quoted and trailing-backslash values", () => RunAsync(async () =>
        {
            using var runner = new CommandRunner();
            string[] values = { "", "plain", "a b", "a\"b", @"C:\folder with spaces\", "\\\"", "\\\\\"tail\\", "& %PATH% $value ; 'quote'", "line one\nline two" };
            var lines = new List<string>();
            await runner.RunAsync(SelfExecutable(), SelfArguments(new[] { "--helper-echo-many" }.Concat(values).ToArray()), root, line => lines.Add(line), default);
            True(values.SequenceEqual(lines.Select(line => Encoding.UTF8.GetString(Convert.FromBase64String(line)))));
        }));
        Test("Completion drains all stdout and stderr lines", () => RunAsync(async () =>
        {
            using var runner = new CommandRunner();
            var lines = new System.Collections.Concurrent.ConcurrentBag<string>();
            await runner.RunAsync(SelfExecutable(), SelfArguments("--helper-logs"), root, lines.Add, default);
            Equal(400, lines.Count);
        }));
        Test("Descendants are stopped even when their parent exits first", () => RunAsync(async () =>
        {
            using var runner = new CommandRunner();
            string directory = NewDirectory();
            var command = runner.Start(SelfExecutable(), SelfArguments("--helper-root-exit", directory), root, _ => { });
            await command.Completion.WaitAsync(TimeSpan.FromSeconds(10));
            Equal(0, command.Completion.Result);
            await WaitUntilGone(int.Parse(File.ReadAllText(Path.Combine(directory, "child.pid"))));
            await WaitUntilGone(int.Parse(File.ReadAllText(Path.Combine(directory, "grandchild.pid"))));
            await runner.StopAsync(command);
        }));
        Test("Non-zero command exit is reported", () => RunAsync(async () =>
        {
            using var runner = new CommandRunner();
            await ThrowsAsync<InvalidOperationException>(() => runner.RunAsync(SelfExecutable(),
                SelfArguments("--helper-fail"), root, _ => { }, default));
        }));
        Test("Cancelling a command stops it", () => RunAsync(async () =>
        {
            using var runner = new CommandRunner();
            using var cancellation = new CancellationTokenSource(300);
            await ThrowsAsync<OperationCanceledException>(() => runner.RunAsync(SelfExecutable(),
                SelfArguments("--helper-wait"), root, _ => { }, cancellation.Token));
        }));
        Test("Stopping a command kills its child and grandchild, not unrelated processes", () => RunAsync(async () =>
        {
            using var runner = new CommandRunner();
            using var unrelated = Process.Start(SelfStart("--helper-wait"))!;
            string directory = NewDirectory();
            var command = runner.Start(SelfExecutable(), SelfArguments("--helper-root", directory), root, _ => { });
            try
            {
                await WaitForFile(Path.Combine(directory, "grandchild.pid"));
                int child = int.Parse(File.ReadAllText(Path.Combine(directory, "child.pid")));
                int grandchild = int.Parse(File.ReadAllText(Path.Combine(directory, "grandchild.pid")));
                await command.DisposeAsync();
                await WaitUntilGone(child);
                await WaitUntilGone(grandchild);
                True(!unrelated.HasExited);
            }
            finally { unrelated.Kill(); await unrelated.WaitForExitAsync(); }
        }));
        Test("Closing the command runner stops all owned commands", () => RunAsync(async () =>
        {
            var runner = new CommandRunner();
            var first = runner.Start(SelfExecutable(), SelfArguments("--helper-wait"), root, _ => { });
            var second = runner.Start(SelfExecutable(), SelfArguments("--helper-wait"), root, _ => { });
            await runner.DisposeAsync();
            True(!first.IsRunning && !second.IsRunning);
            Throws<ObjectDisposedException>(() => runner.Start(SelfExecutable(), Array.Empty<string>(), root, _ => { }));
        }));
        Test("An occupied viewer port is not reused or terminated", () =>
        {
            var listener = new System.Net.Sockets.TcpListener(System.Net.IPAddress.Loopback, 0);
            listener.Start();
            try { Throws<IOException>(() => SplattingTools.CheckViewerPort(((System.Net.IPEndPoint)listener.LocalEndpoint).Port)); }
            finally { listener.Stop(); }
        });
        int realPcdIndex = Array.IndexOf(args, "--real-pcd");
        if (realPcdIndex >= 0 && realPcdIndex + 1 < args.Length)
            Test("The supplied VISTA RoomMap converts without losing any point text", () =>
            {
                string input = args[realPcdIndex + 1];
                string output = Path.Combine(root, Path.GetFileNameWithoutExtension(input) + ".txt");
                long count = PcdTextConverter.Convert(input, output);
                string[] original = File.ReadAllLines(input);
                int start = Array.FindIndex(original, line => line.Trim().Equals("DATA ascii", StringComparison.OrdinalIgnoreCase)) + 1;
                string[] body = original.Skip(start).Where(line => !string.IsNullOrWhiteSpace(line)).ToArray();
                Equal((long)body.Length, count);
                True(body.SequenceEqual(File.ReadAllLines(output)));
                Console.WriteLine($"  Real PCD: {count} points preserved.");
            });

        int integrationIndex = Array.IndexOf(args, "--integration");
        if (integrationIndex >= 0 && integrationIndex + 1 < args.Length && realPcdIndex >= 0)
            Test("Real uv sync, npm ci, Gaussian generation, HTTP viewer and shutdown", () => RunAsync(async () =>
            {
                string repository = Path.GetFullPath(args[integrationIndex + 1]);
                RepositoryValidator.Validate(repository);
                var tools = SplattingTools.Find();
                using var runner = new CommandRunner();
                using var timeout = new CancellationTokenSource(TimeSpan.FromMinutes(8));
                Action<string> log = line => Console.WriteLine("  " + line);
                await tools.SetupAsync(runner, repository, log, timeout.Token);
                string txt = Path.Combine(repository, "integration input & map.txt");
                PcdTextConverter.Convert(args[realPcdIndex + 1], txt, true);
                string ply = Path.Combine(repository, "out", "integration-optimized.ply");
                Directory.CreateDirectory(Path.GetDirectoryName(ply)!);
                await tools.GenerateAsync(runner, repository, txt, ply, log, timeout.Token);
                foreach (string name in new[] { "scene.ply", "uniform.ply", "surface.ply", "points.ply" })
                    True(new FileInfo(Path.Combine(repository, "public", name)).Length > 0);
                var listener = new System.Net.Sockets.TcpListener(System.Net.IPAddress.Loopback, 0);
                listener.Start();
                int port = ((System.Net.IPEndPoint)listener.LocalEndpoint).Port;
                listener.Stop();
                var viewer = tools.StartViewer(runner, repository, port, log);
                try
                {
                    // First access to a newly staged asset may trigger Vite/AV cold work.
                    using var http = new System.Net.Http.HttpClient { Timeout = TimeSpan.FromSeconds(10) };
                    bool ready = false;
                    for (int attempt = 0; attempt < 100; attempt++)
                    {
                        if (!viewer.IsRunning) throw new Exception("Viewer exited early.");
                        try
                        {
                            using var reply = await http.GetAsync(SplattingTools.ViewerUrl(port), timeout.Token);
                            reply.EnsureSuccessStatusCode();
                            string html = await reply.Content.ReadAsStringAsync();
                            if (html.Contains("/src/viewer/main.tsx")) { ready = true; break; }
                        }
                        catch (System.Net.Http.HttpRequestException) { }
                        catch (TaskCanceledException) when (!timeout.IsCancellationRequested) { }
                        await Task.Delay(100, timeout.Token);
                    }
                    True(ready);
                    using var asset = await http.GetAsync(SplattingTools.ViewerUrl(port) + "scene.ply", timeout.Token);
                    asset.EnsureSuccessStatusCode();
                    byte[] bytes = await asset.Content.ReadAsByteArrayAsync();
                    True(bytes.Length > 100);
                }
                finally { await runner.StopAsync(viewer); }
                // Job termination schedules every descendant for exit; allow Windows
                // to finish releasing the listener before checking it.
                for (int attempt = 0; ; attempt++)
                {
                    try { SplattingTools.CheckViewerPort(port); break; }
                    catch (IOException) when (attempt < 60) { await Task.Delay(50); }
                }
            }));

        Test("WPF layout loads with TXT dropdown and English controls", () =>
        {
            // Layout/interaction tests run without a top-level application loop.
            var app = new Application { ShutdownMode = ShutdownMode.OnExplicitShutdown };
            SynchronizationContext.SetSynchronizationContext(new DispatcherSynchronizationContext(Dispatcher.CurrentDispatcher));
            var window = CreateWindow(FakeRepository(Path.Combine(root, "UI repository")));
            var content = (FrameworkElement)window.Content;
            content.Measure(new Size(996, 680));
            content.Arrange(new Rect(0, 0, 996, 680));
            content.UpdateLayout();
            Dispatcher.CurrentDispatcher.Invoke(() => { }, DispatcherPriority.ApplicationIdle);
            var format = (ComboBox)window.FindName("FormatBox");
            Equal(1, format.Items.Count);
            Equal(".txt", ((ComboBoxItem)format.Items[0]).Content.ToString());
            Equal("Convert", ((Button)window.FindName("ConvertButton")).Content.ToString());
            Equal("5173", ((TextBox)window.FindName("ViewerPortBox")).Text);
            True(((Button)window.FindName("RepositoryButton")).IsEnabled);
            var bitmap = new RenderTargetBitmap(996, 680, 96, 96, PixelFormats.Pbgra32);
            bitmap.Render(content);
            var encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(bitmap));
            using var image = File.Create(Path.Combine(root, "wpf-preview.png"));
            encoder.Save(image);
            Console.WriteLine("  WPF preview: " + Path.Combine(root, "wpf-preview.png"));

            Test("The window opens without a repository or a configuration file", () =>
            {
                var empty = CreateWindow();
                Equal("", ((TextBox)empty.FindName("RepositoryBox")).Text);
                True(((Button)empty.FindName("ConvertButton")).IsEnabled);
                True(((Button)empty.FindName("RepositoryButton")).IsEnabled);
                foreach (string button in new[] { "CheckButton", "SetupButton", "GenerateButton", "StartButton" })
                    True(!((Button)empty.FindName(button)).IsEnabled);
                True(!File.Exists(Path.Combine(root, "SplattingConfig.txt")));
                CloseWindow(empty);
            });
            Test("Convert works without selecting a Gaussian repository", () =>
            {
                var converter = CreateWindow();
                string source = Source(Header + Data);
                ((TextBox)converter.FindName("InputBox")).Text = source;
                ((Button)converter.FindName("ConvertButton")).RaiseEvent(new RoutedEventArgs(Button.ClickEvent));
                var task = (Task)typeof(MainWindow).GetField("operation", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(converter)!;
                Pump(task.WaitAsync(TimeSpan.FromSeconds(5)));
                Equal(2, File.ReadAllLines(Path.ChangeExtension(source, ".txt")).Length);
                Equal("", ((TextBox)converter.FindName("RepositoryBox")).Text);
                CloseWindow(converter);
            });
            Test("Selecting a valid repository enables Gaussian controls without writing a config", () =>
            {
                var selection = CreateWindow();
                string repo = FakeRepository(NewDirectory());
                string[] before = Directory.GetFiles(repo, "*", SearchOption.AllDirectories);
                Pump(selection.SelectRepositoryAsync(repo));
                Equal(repo, ((TextBox)selection.FindName("RepositoryBox")).Text);
                foreach (string button in new[] { "CheckButton", "SetupButton", "GenerateButton", "StartButton" })
                    True(((Button)selection.FindName(button)).IsEnabled);
                True(before.SequenceEqual(Directory.GetFiles(repo, "*", SearchOption.AllDirectories)));
                CloseWindow(selection);
            });
            Test("Invalid repository selection preserves the current viewer; a valid switch stops it", () =>
            {
                string firstRepo = FakeRepository(NewDirectory());
                string nextRepo = FakeRepository(NewDirectory());
                var selection = CreateWindow(firstRepo);
                var flags = BindingFlags.Instance | BindingFlags.NonPublic;
                var runner = (CommandRunner)typeof(MainWindow).GetField("runner", flags)!.GetValue(selection)!;
                string directory = NewDirectory();
                var owned = runner.Start(SelfExecutable(), SelfArguments("--helper-root", directory), root, _ => { });
                typeof(MainWindow).GetField("viewer", flags)!.SetValue(selection, owned);
                Pump(WaitForFile(Path.Combine(directory, "grandchild.pid")));
                int child = int.Parse(File.ReadAllText(Path.Combine(directory, "child.pid")));
                int grandchild = int.Parse(File.ReadAllText(Path.Combine(directory, "grandchild.pid")));
                Throws<InvalidDataException>(() => Pump(selection.SelectRepositoryAsync(NewDirectory())));
                True(owned.IsRunning);
                Equal(firstRepo, ((TextBox)selection.FindName("RepositoryBox")).Text);
                Pump(selection.SelectRepositoryAsync(nextRepo));
                True(!owned.IsRunning);
                Pump(WaitUntilGone(child));
                Pump(WaitUntilGone(grandchild));
                Equal(nextRepo, ((TextBox)selection.FindName("RepositoryBox")).Text);
                CloseWindow(selection);
            });

            Test("Reopening restores the last chosen repository and the folder-picker position", () =>
            {
                var history = NewHistory();
                string repo = FakeRepository(NewDirectory());
                var selection = CreateWindow(history: history);
                Pump(selection.SelectRepositoryAsync(repo));
                CloseWindow(selection);
                var reopened = CreateWindow(history: history);
                Equal(repo, ((TextBox)reopened.FindName("RepositoryBox")).Text);
                Equal(repo, typeof(MainWindow).GetMethod("GetRepositoryDialogFolder", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(reopened, null));
                True(((Button)reopened.FindName("GenerateButton")).IsEnabled);
                True(!((Button)reopened.FindName("StopButton")).IsEnabled);
                True(typeof(MainWindow).GetField("operation", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(reopened) == null);
                CloseWindow(reopened);
            });
            Test("A moved saved repository does not block startup or plain conversion", () =>
            {
                var history = NewHistory();
                string parent = NewDirectory();
                string original = FakeRepository(Path.Combine(parent, "repo"));
                string moved = Path.Combine(parent, "repo-moved");
                history.Save(original);
                Directory.Move(original, moved);
                var reopened = CreateWindow(history: history);
                Equal("", ((TextBox)reopened.FindName("RepositoryBox")).Text);
                True(((Button)reopened.FindName("ConvertButton")).IsEnabled);
                True(!((Button)reopened.FindName("GenerateButton")).IsEnabled);
                True(((TextBlock)reopened.FindName("StatusText")).Text.Contains("unavailable"));
                Equal(parent, typeof(MainWindow).GetMethod("GetRepositoryDialogFolder", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(reopened, null));
                Pump(reopened.SelectRepositoryAsync(moved));
                Equal(moved, history.Load());
                CloseWindow(reopened);
            });
            Test("Selecting the current repository also remembers it", () =>
            {
                var history = NewHistory();
                string repo = FakeRepository(NewDirectory());
                var selection = CreateWindow(repo, history);
                Pump(selection.SelectRepositoryAsync(repo));
                Equal(repo, history.Load());
                CloseWindow(selection);
            });

            Test("Invalid viewer ports block viewer/generation but not plain conversion", () =>
            {
                var selection = CreateWindow(FakeRepository(NewDirectory()));
                var portBox = (TextBox)selection.FindName("ViewerPortBox");
                foreach (string invalid in new[] { "", "bad", "1023", "65536" })
                {
                    portBox.Text = invalid;
                    True(!((Button)selection.FindName("StartButton")).IsEnabled);
                    True(!((Button)selection.FindName("GenerateButton")).IsEnabled);
                    True(((Button)selection.FindName("ConvertButton")).IsEnabled);
                    True(((TextBlock)selection.FindName("ViewerPortHint")).Text.Contains("1024"));
                }
                portBox.Text = "5180";
                True(((Button)selection.FindName("StartButton")).IsEnabled);
                True(((Button)selection.FindName("GenerateButton")).IsEnabled);
                CloseWindow(selection);
            });
            Test("Closing and reopening restores the edited viewer port without starting tools", () =>
            {
                var history = NewHistory();
                var selection = CreateWindow(history: history);
                ((TextBox)selection.FindName("ViewerPortBox")).Text = "5180";
                CloseWindow(selection);
                var reopened = CreateWindow(history: history);
                Equal("5180", ((TextBox)reopened.FindName("ViewerPortBox")).Text);
                True(((TextBox)reopened.FindName("ViewerPortBox")).IsEnabled);
                True(typeof(MainWindow).GetField("viewer", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(reopened) == null);
                CloseWindow(reopened);
                var invalid = CreateWindow(history: history);
                ((TextBox)invalid.FindName("ViewerPortBox")).Text = "bad";
                CloseWindow(invalid);
                Equal(5180, history.LoadViewerPort());
            });
            Test("Leaving the port field remembers a valid value immediately", () =>
            {
                var history = NewHistory();
                var selection = CreateWindow(history: history);
                ((TextBox)selection.FindName("ViewerPortBox")).Text = "5181";
                typeof(MainWindow).GetMethod("ViewerPort_LostFocus", BindingFlags.Instance | BindingFlags.NonPublic)!
                    .Invoke(selection, new object?[] { selection, null });
                Equal(5181, history.LoadViewerPort());
                CloseWindow(selection);
            });
            Test("Viewer uses the selected port, locks edits, and unlocks after stopping", () =>
            {
                string repo = FakeRepository(NewDirectory());
                Directory.CreateDirectory(Path.Combine(repo, "node_modules", "vite", "bin"));
                File.WriteAllText(Path.Combine(repo, "node_modules", "vite", "bin", "vite.js"), "");
                var history = NewHistory();
                var selection = CreateWindow(repo, history);
                var portBox = (TextBox)selection.FindName("ViewerPortBox");
                int port = FreePort();
                portBox.Text = port.ToString(CultureInfo.InvariantCulture);
                var flags = BindingFlags.Instance | BindingFlags.NonPublic;
                try
                {
                    var tools = new SplattingTools(SelfExecutable(), "--helper-viewer", "unused");
                    Pump(((Task)typeof(MainWindow).GetMethod("StartViewerAsync", flags)!
                        .Invoke(selection, new object[] { tools, CancellationToken.None })!).WaitAsync(TimeSpan.FromSeconds(15)));
                    True(!portBox.IsEnabled);
                    Equal("Viewer: " + SplattingTools.ViewerUrl(port), ((TextBlock)selection.FindName("ViewerText")).Text);
                    Equal(port, (int)typeof(MainWindow).GetField("viewerPort", flags)!.GetValue(selection)!);
                    Equal(port, history.LoadViewerPort());
                    True(((Button)selection.FindName("OpenButton")).IsEnabled);
                    Throws<IOException>(() => SplattingTools.CheckViewerPort(port));
                    Pump((Task)typeof(MainWindow).GetMethod("StopViewerAsync", flags)!.Invoke(selection, null)!);
                    True(portBox.IsEnabled);
                    True(!((Button)selection.FindName("OpenButton")).IsEnabled);
                    portBox.Text = FreePort().ToString(CultureInfo.InvariantCulture);
                    Pump(((Task)typeof(MainWindow).GetMethod("StartViewerAsync", flags)!
                        .Invoke(selection, new object[] { tools, CancellationToken.None })!).WaitAsync(TimeSpan.FromSeconds(15)));
                    Equal("Viewer: " + SplattingTools.ViewerUrl(int.Parse(portBox.Text)), ((TextBlock)selection.FindName("ViewerText")).Text);
                }
                finally { CloseWindow(selection); }
            });
            Test("A busy selected port leaves its owner running and the port input editable", () =>
            {
                string repo = FakeRepository(NewDirectory());
                Directory.CreateDirectory(Path.Combine(repo, "node_modules", "vite", "bin"));
                File.WriteAllText(Path.Combine(repo, "node_modules", "vite", "bin", "vite.js"), "");
                var selection = CreateWindow(repo);
                var listener = new System.Net.Sockets.TcpListener(System.Net.IPAddress.Loopback, 0);
                listener.Start();
                int port = ((System.Net.IPEndPoint)listener.LocalEndpoint).Port;
                ((TextBox)selection.FindName("ViewerPortBox")).Text = port.ToString(CultureInfo.InvariantCulture);
                try
                {
                    var tools = new SplattingTools(SelfExecutable(), "--helper-viewer", "unused");
                    Throws<IOException>(() => Pump((Task)typeof(MainWindow).GetMethod("StartViewerAsync", BindingFlags.Instance | BindingFlags.NonPublic)!
                        .Invoke(selection, new object[] { tools, CancellationToken.None })!));
                    Equal(port, ((System.Net.IPEndPoint)listener.LocalEndpoint).Port);
                    True(((TextBox)selection.FindName("ViewerPortBox")).IsEnabled);
                    True(!((Button)selection.FindName("OpenButton")).IsEnabled);
                }
                finally { listener.Stop(); CloseWindow(selection); }
            });
            Test("Port input is locked during an operation and restored afterward", () =>
            {
                var selection = CreateWindow();
                var flags = BindingFlags.Instance | BindingFlags.NonPublic;
                var entered = new TaskCompletionSource<bool>();
                var release = new TaskCompletionSource<bool>();
                Func<CancellationToken, Task> action = async _ => { entered.SetResult(true); await release.Task; };
                Task operation = (Task)typeof(MainWindow).GetMethod("ExecuteAsync", flags)!.Invoke(selection, new object[] { action })!;
                Pump(entered.Task);
                True(!((TextBox)selection.FindName("ViewerPortBox")).IsEnabled);
                release.SetResult(true);
                Pump(operation);
                True(((TextBox)selection.FindName("ViewerPortBox")).IsEnabled);
                CloseWindow(selection);
            });

            Test("Closing the WPF window automatically stops its viewer tree", () =>
            {
                var runner = (CommandRunner)typeof(MainWindow).GetField("runner", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(window)!;
                string directory = NewDirectory();
                var owned = runner.Start(SelfExecutable(), SelfArguments("--helper-root", directory), root, line => Console.WriteLine("  WPF helper: " + line));
                typeof(MainWindow).GetField("viewer", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(window, owned);
                Pump(WaitForFile(Path.Combine(directory, "grandchild.pid")));
                int child = int.Parse(File.ReadAllText(Path.Combine(directory, "child.pid")));
                int grandchild = int.Parse(File.ReadAllText(Path.Combine(directory, "grandchild.pid")));
                var closed = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
                window.Closed += (_, e) => closed.TrySetResult(true);
                window.Close();
                Pump(closed.Task.WaitAsync(TimeSpan.FromSeconds(10)));
                Pump(WaitUntilGone(child));
                Pump(WaitUntilGone(grandchild));
                True(!owned.IsRunning);
            });
            Test("Closing an idle WPF window completes without recursive-close errors", () =>
            {
                var idle = CreateWindow(repository: Path.Combine(root, "UI repository"));
                var closed = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
                idle.Closed += (_, e) => closed.TrySetResult(true);
                idle.Close();
                Pump(closed.Task.WaitAsync(TimeSpan.FromSeconds(5)));
            });
            Test("Closing WPF cancels an active command and waits for cleanup", () =>
            {
                var active = CreateWindow(Path.Combine(root, "UI repository"));
                var flags = BindingFlags.Instance | BindingFlags.NonPublic;
                var activeRunner = (CommandRunner)typeof(MainWindow).GetField("runner", flags)!.GetValue(active)!;
                var cancellation = new CancellationTokenSource();
                Task task = activeRunner.RunAsync(SelfExecutable(), SelfArguments("--helper-wait"), root, _ => { }, cancellation.Token);
                typeof(MainWindow).GetField("cancellation", flags)!.SetValue(active, cancellation);
                typeof(MainWindow).GetField("operation", flags)!.SetValue(active, task);
                var closed = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
                active.Closed += (_, e) => closed.TrySetResult(true);
                active.Close();
                Pump(closed.Task.WaitAsync(TimeSpan.FromSeconds(10)));
                True(cancellation.IsCancellationRequested && task.IsCanceled);
                cancellation.Dispose();
            });
        });
        foreach (string keyPath in historyKeys)
        {
            // These keys were uniquely generated by this test run; never touch the
            // application's production LastRepositoryPath value or other user settings.
            if (!keyPath.StartsWith(RepositoryHistory.DefaultKeyPath + @"\Tests\", StringComparison.Ordinal))
                throw new InvalidOperationException("Unsafe test preference cleanup path.");
            Registry.CurrentUser.DeleteSubKeyTree(keyPath, false);
        }
        Console.WriteLine($"\n{passed} passed; {failed} failed.");
        return failed == 0 ? 0 : 1;
    }

    private static string NewDirectory()
    {
        string directory = Path.Combine(root, Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        return directory;
    }
    private static string NewHistoryKey()
    {
        string keyPath = RepositoryHistory.DefaultKeyPath + @"\Tests\" + Guid.NewGuid().ToString("N");
        historyKeys.Add(keyPath);
        return keyPath;
    }
    private static RepositoryHistory NewHistory() => new RepositoryHistory(NewHistoryKey());
    private static MainWindow CreateWindow(string repository = "", RepositoryHistory? history = null) =>
        new MainWindow(repository, history ?? NewHistory());
    private static string FakeRepository(string directory)
    {
        Directory.CreateDirectory(Path.Combine(directory, "python", "lidar_splat_preprocess"));
        foreach (string file in new[] { "package.json", "package-lock.json", "pyproject.toml", "uv.lock", "python/lidar_splat_preprocess/cli.py" })
            File.WriteAllText(Path.Combine(directory, file), "");
        return directory;
    }
    private static string Source(string content, string name = "map.pcd")
    {
        string source = Path.Combine(NewDirectory(), name);
        File.WriteAllText(source, content);
        return source;
    }
    private static void Test(string name, Action action)
    {
        try { action(); passed++; Console.WriteLine("PASS " + name); }
        catch (Exception ex) { failed++; Console.WriteLine("FAIL " + name + "\n" + ex); }
    }
    private static void True(bool condition) { if (!condition) throw new Exception("Assertion failed."); }
    private static void Equal<T>(T expected, T actual) { if (!Equals(expected, actual)) throw new Exception($"Expected {expected}; actual {actual}."); }
    private static void Throws<T>(Action action) where T : Exception
    {
        try { action(); } catch (T) { return; }
        throw new Exception("Expected " + typeof(T).Name);
    }
    private static async Task ThrowsAsync<T>(Func<Task> action) where T : Exception
    {
        try { await action(); } catch (T) { return; }
        throw new Exception("Expected " + typeof(T).Name);
    }
    private static void RunAsync(Func<Task> action) => action().GetAwaiter().GetResult();
    private static void CloseWindow(Window window)
    {
        var closed = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        window.Closed += (_, e) => closed.TrySetResult(true);
        window.Close();
        Pump(closed.Task.WaitAsync(TimeSpan.FromSeconds(5)));
    }
    private static void Pump(Task task)
    {
        if (!task.IsCompleted)
        {
            var frame = new DispatcherFrame();
            var dispatcher = Dispatcher.CurrentDispatcher;
            _ = task.ContinueWith(_ => dispatcher.BeginInvoke(new Action(() => frame.Continue = false)), TaskScheduler.Default);
            Dispatcher.PushFrame(frame);
        }
        task.GetAwaiter().GetResult();
    }
    private static string SelfExecutable() => Assembly.GetExecutingAssembly().Location;
    private static int FreePort()
    {
        var listener = new System.Net.Sockets.TcpListener(System.Net.IPAddress.Loopback, 0);
        listener.Start();
        int port = ((System.Net.IPEndPoint)listener.LocalEndpoint).Port;
        listener.Stop();
        return port;
    }
    private static string[] SelfArguments(params string[] arguments) => arguments;
    private static ProcessStartInfo SelfStart(params string[] arguments)
    {
        var start = new ProcessStartInfo(SelfExecutable()) { UseShellExecute = false, CreateNoWindow = true };
        start.Arguments = Compatibility.QuoteArguments(SelfArguments(arguments));
        return start;
    }
    private static int Helper(string[] args)
    {
        switch (args[0])
        {
            case "--helper-viewer":
                int port = int.Parse(args[Array.IndexOf(args, "--port") + 1], CultureInfo.InvariantCulture);
                var listener = new System.Net.Sockets.TcpListener(System.Net.IPAddress.Loopback, port);
                listener.Start();
                while (true)
                {
                    using var client = listener.AcceptTcpClient();
                    using var stream = client.GetStream();
                    using var reader = new StreamReader(stream, Encoding.ASCII, false, 1024, true);
                    while (!string.IsNullOrEmpty(reader.ReadLine())) { }
                    byte[] response = Encoding.ASCII.GetBytes("HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nOK");
                    stream.Write(response, 0, response.Length);
                }
            case "--helper-echo": Console.WriteLine(Convert.ToBase64String(Encoding.UTF8.GetBytes(args[1]))); return 0;
            case "--helper-echo-many":
                foreach (string value in args.Skip(1)) Console.WriteLine(Convert.ToBase64String(Encoding.UTF8.GetBytes(value)));
                return 0;
            case "--helper-logs":
                for (int i = 0; i < 200; i++) { Console.WriteLine("out " + i); Console.Error.WriteLine("err " + i); }
                return 0;
            case "--helper-fail": return 7;
            case "--helper-root": Process.Start(SelfStart("--helper-child", args[1])); break;
            case "--helper-root-exit":
                Process.Start(SelfStart("--helper-child", args[1]));
                WaitForFile(Path.Combine(args[1], "grandchild.pid")).GetAwaiter().GetResult();
                return 0;
            case "--helper-child":
                File.WriteAllText(Path.Combine(args[1], "child.pid"), Process.GetCurrentProcess().Id.ToString());
                Process.Start(SelfStart("--helper-grandchild", args[1])); break;
            case "--helper-grandchild": File.WriteAllText(Path.Combine(args[1], "grandchild.pid"), Process.GetCurrentProcess().Id.ToString()); break;
        }
        Thread.Sleep(Timeout.Infinite);
        return 0;
    }
    private static async Task WaitForFile(string path)
    {
        for (int i = 0; i < 100; i++) { if (File.Exists(path) && new FileInfo(path).Length > 0) return; await Task.Delay(50); }
        throw new Exception("Child process did not start: " + path);
    }
    private static async Task WaitUntilGone(int pid)
    {
        for (int i = 0; i < 100; i++)
        {
            try { using var process = Process.GetProcessById(pid); if (process.HasExited) return; }
            catch (ArgumentException) { return; }
            await Task.Delay(50);
        }
        throw new Exception("Owned child process is still running: " + pid);
    }
    private sealed class ImmediateProgress : IProgress<long>
    {
        private readonly Action<long> report;
        public ImmediateProgress(Action<long> report) { this.report = report; }
        public void Report(long value) => report(value);
    }
}
