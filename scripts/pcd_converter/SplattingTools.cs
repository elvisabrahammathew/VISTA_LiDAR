using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Diagnostics;
using System.IO;
using System.Globalization;
using System.Net.NetworkInformation;

namespace PcdConverter
{

public sealed class SplattingTools
{
    public const int DefaultViewerPort = 5173;

    public static bool TryParseViewerPort(string text, out int port) =>
        int.TryParse(text.Trim(), NumberStyles.None, CultureInfo.InvariantCulture, out port) &&
        port >= 1024 && port <= 65535;

    public static int ParseViewerPort(string text)
    {
        if (!TryParseViewerPort(text, out int port))
            throw new ArgumentException("Viewer Port must be a whole number from 1024 to 65535.");
        return port;
    }

    public string Node { get; }
    public string NpmCli { get; }
    public string Uv { get; }

    public SplattingTools(string node, string npmCli, string uv)
    {
        Node = node;
        NpmCli = npmCli;
        Uv = uv;
    }
    public static SplattingTools Find()
    {
        string node = FindExecutable("node.exe", Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles), "nodejs"));
        string uv = FindExecutable("uv.exe", Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".local", "bin"),
            Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".cargo", "bin"));
        string? npm = new[] { Path.GetDirectoryName(node)! }.Concat(SearchDirectories())
            .Select(directory => Path.Combine(directory, "node_modules", "npm", "bin", "npm-cli.js"))
            .FirstOrDefault(File.Exists);
        if (npm is null) throw new FileNotFoundException("npm was not found. Install Node.js with npm, then restart this application.");
        return new SplattingTools(node, npm, uv);
    }

    public static string ViewerUrl(int port) => $"http://127.0.0.1:{port}/";

    public static void CheckViewerPort(int port)
    {
        if (port < 1024 || port > 65535)
            throw new ArgumentOutOfRangeException(nameof(port), "Viewer Port must be from 1024 to 65535.");
        // A bind probe can report a false conflict while Windows retains TIME_WAIT
        // connections after stopping Vite. Check LISTEN sockets; --strictPort remains
        // the authoritative check if another application wins the startup race.
        if (IPGlobalProperties.GetIPGlobalProperties().GetActiveTcpListeners().Any(endpoint => endpoint.Port == port))
            throw new IOException($"Viewer port {port} is already in use. Choose another Viewer Port, or stop the application using it.");
    }

    public async Task CheckAsync(CommandRunner runner, string repository, Action<string> log, CancellationToken token)
    {
        string version = "";
        await runner.RunAsync(Node, new[] { "--version" }, repository, line => { version += line; log(line); }, token);
        if (!Version.TryParse(version.Trim().TrimStart('v'), out var parsed) ||
            parsed.Major < 22 || (parsed.Major == 22 && parsed.Minor < 12))
            throw new InvalidOperationException("Node.js 22.12 or newer is required by this repository's viewer.");
        await NpmAsync(runner, repository, new[] { "--version" }, log, token);
        await runner.RunAsync(Uv, new[] { "--version" }, repository, log, token);
    }

    public Task NpmAsync(CommandRunner runner, string repository, string[] arguments, Action<string> log, CancellationToken token)
    {
        log("npm " + string.Join(" ", arguments));
        // Same npm CLI as npm.cmd, without passing paths through cmd.exe or PowerShell.
        return runner.RunAsync(Node, new[] { NpmCli }.Concat(arguments), repository, log, token);
    }

    public async Task SetupAsync(CommandRunner runner, string repository, Action<string> log, CancellationToken token)
    {
        await CheckAsync(runner, repository, log, token);
        log("uv sync --locked --python 3.12");
        await runner.RunAsync(Uv, new[] { "sync", "--locked", "--python", "3.12" }, repository, log, token);
        await NpmAsync(runner, repository, new[] { "ci" }, log, token);
    }

    public async Task GenerateAsync(CommandRunner runner, string repository, string input, string output,
        Action<string> log, CancellationToken token)
    {
        log($"Generating Gaussian files from {input}");
        await runner.RunAsync(Uv, new[] { "run", "--locked", "--python", "3.12", "lidar-splat", "convert", input, output,
            "--intensity-encoding", "uint8", "--stage" }, repository, log, token);
    }

    public RunningCommand StartViewer(CommandRunner runner, string repository, int port, Action<string> log)
    {
        if (!File.Exists(Path.Combine(repository, "node_modules", "vite", "bin", "vite.js")))
            throw new IOException("Viewer dependencies are missing. Run Setup / Repair Environment first.");
        CheckViewerPort(port);
        log($"npm run dev -- --host 127.0.0.1 --port {port} --strictPort");
        return runner.Start(Node, new[] { NpmCli, "run", "dev", "--", "--host", "127.0.0.1", "--port", port.ToString(), "--strictPort" }, repository, log);
    }

    private static IEnumerable<string> SearchDirectories() =>
        (Environment.GetEnvironmentVariable("PATH") ?? "").Split(Path.PathSeparator)
        .Select(path => path.Trim().Trim('"')).Where(path => path.Length > 0);

    private static string FindExecutable(string name, params string[] fallbackDirectories)
    {
        string? result = SearchDirectories().Concat(fallbackDirectories)
            .Select(directory => Path.Combine(directory, name)).FirstOrDefault(File.Exists);
        return result ?? throw new FileNotFoundException($"{name} was not found. Install the required tool and restart this application.");
    }
}
}
