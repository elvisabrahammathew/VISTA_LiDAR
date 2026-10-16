using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace PcdConverter
{

// Each command has its own Windows Job. Closing it also stops npm's Node/Vite children.
// Browsers are opened separately, so they never belong to one of these jobs.
public sealed class CommandRunner : IDisposable
{
    private readonly HashSet<RunningCommand> commands = new HashSet<RunningCommand>();
    private bool disposed;

    public RunningCommand Start(string executable, IEnumerable<string> arguments, string directory, Action<string> log)
    {
        if (disposed) throw new ObjectDisposedException(nameof(CommandRunner));
        var command = new RunningCommand(executable, arguments, directory, log);
        commands.Add(command);
        return command;
    }

    public async Task RunAsync(string executable, IEnumerable<string> arguments, string directory,
        Action<string> log, CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var command = Start(executable, arguments, directory, log);
        try
        {
            int exitCode = await command.Completion.WaitAsync(cancellationToken);
            if (exitCode != 0) throw new InvalidOperationException($"Command failed (exit code {exitCode}). See Execution Log for details.");
        }
        finally
        {
            await StopAsync(command);
        }
    }

    public async Task StopAsync(RunningCommand command)
    {
        await command.DisposeAsync();
        commands.Remove(command);
    }

    public async Task DisposeAsync()
    {
        disposed = true;
        foreach (var command in commands.ToArray()) await command.DisposeAsync();
        commands.Clear();
    }

    public void Dispose() => DisposeAsync().GetAwaiter().GetResult();
}

public sealed class RunningCommand : IDisposable
{
    private readonly Process process;
    private readonly SafeFileHandle job;
    private bool disposed;
    public Task<int> Completion { get; }
    public bool IsRunning => !Completion.IsCompleted;

    internal RunningCommand(string executable, IEnumerable<string> arguments, string directory, Action<string> log)
    {
        job = CreateJobObject(IntPtr.Zero, null);
        if (job.IsInvalid) throw new Win32Exception(Marshal.GetLastWin32Error());
        var limits = new JobLimits();
        limits.BasicLimitInformation.LimitFlags = 0x2000; // JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if (!SetInformationJobObject(job, 9, ref limits, (uint)Marshal.SizeOf<JobLimits>()))
        {
            job.Dispose();
            throw new Win32Exception(Marshal.GetLastWin32Error());
        }
        var start = new ProcessStartInfo(executable)
        {
            Arguments = Compatibility.QuoteArguments(arguments),
            WorkingDirectory = directory,
            UseShellExecute = false,
            CreateNoWindow = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true
        };
        start.EnvironmentVariables["NO_COLOR"] = "1";
        start.EnvironmentVariables["PYTHONUNBUFFERED"] = "1";
        process = new Process { StartInfo = start };
        var exited = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        var outputEnded = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        var errorEnded = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        process.EnableRaisingEvents = true;
        process.Exited += (_, e) => exited.TrySetResult(true);
        process.OutputDataReceived += (_, e) => { if (e.Data != null) log(e.Data); else outputEnded.TrySetResult(true); };
        process.ErrorDataReceived += (_, e) => { if (e.Data != null) log(e.Data); else errorEnded.TrySetResult(true); };
        try
        {
            process.Start();
            if (!AssignProcessToJobObject(job, process.Handle))
                throw new Win32Exception(Marshal.GetLastWin32Error(), "Unable to supervise the command's process tree.");
            process.BeginOutputReadLine();
            process.BeginErrorReadLine();
            if (process.HasExited) exited.TrySetResult(true);
            Completion = WaitAsync(exited.Task, outputEnded.Task, errorEnded.Task);
        }
        catch
        {
            job.Dispose();
            try { if (!process.HasExited) process.Kill(); } catch (InvalidOperationException) { }
            process.Dispose();
            throw;
        }
    }

    private async Task<int> WaitAsync(Task exited, Task outputEnded, Task errorEnded)
    {
        try
        {
            await exited.ConfigureAwait(false);
            // Kill orphaned descendants before waiting for their inherited stdout
            // handles to close. The completion does not depend on the UI thread.
            job.Dispose();
            await Task.WhenAll(outputEnded, errorEnded).ConfigureAwait(false);
            return process.ExitCode;
        }
        finally { job.Dispose(); }
    }

    public async Task DisposeAsync()
    {
        if (disposed) return;
        disposed = true;
        job.Dispose();
        try { if (!process.HasExited) process.Kill(); } catch (InvalidOperationException) { }
        try { await Completion.ConfigureAwait(false); }
        finally { process.Dispose(); }
    }

    public void Dispose() => DisposeAsync().GetAwaiter().GetResult();

    [StructLayout(LayoutKind.Sequential)]
    private struct BasicLimits
    {
        public long PerProcessUserTimeLimit, PerJobUserTimeLimit;
        public uint LimitFlags;
        public UIntPtr MinimumWorkingSetSize, MaximumWorkingSetSize;
        public uint ActiveProcessLimit;
        public UIntPtr Affinity;
        public uint PriorityClass, SchedulingClass;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct IoCounters
    {
        public ulong ReadOperationCount, WriteOperationCount, OtherOperationCount;
        public ulong ReadTransferCount, WriteTransferCount, OtherTransferCount;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct JobLimits
    {
        public BasicLimits BasicLimitInformation;
        public IoCounters IoInfo;
        public UIntPtr ProcessMemoryLimit, JobMemoryLimit, PeakProcessMemoryUsed, PeakJobMemoryUsed;
    }
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    private static extern SafeFileHandle CreateJobObject(IntPtr attributes, string? name);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool SetInformationJobObject(SafeFileHandle job, int informationClass, ref JobLimits information, uint length);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool AssignProcessToJobObject(SafeFileHandle job, IntPtr process);
}
}
