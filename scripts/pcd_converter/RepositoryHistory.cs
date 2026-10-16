using System;
using System.IO;
using Microsoft.Win32;

namespace PcdConverter
{
    // Per-Windows-user preference, shared by Debug and published builds.
    // No settings file is written beside the executable or into the selected repo.
    public sealed class RepositoryHistory
    {
        public const string DefaultKeyPath = @"Software\VISTA\PcdConverter";
        private const string ValueName = "LastRepositoryPath";
        private readonly string keyPath;

        public RepositoryHistory(string keyPath = DefaultKeyPath)
        {
            if (string.IsNullOrWhiteSpace(keyPath) ||
                !(keyPath.Equals(DefaultKeyPath, StringComparison.OrdinalIgnoreCase) ||
                  keyPath.StartsWith(DefaultKeyPath + "\\", StringComparison.OrdinalIgnoreCase)))
                throw new ArgumentException("The preference key must belong to this application.", nameof(keyPath));
            this.keyPath = keyPath;
        }

        public string Load()
        {
            using var key = Registry.CurrentUser.OpenSubKey(keyPath);
            return key?.GetValue(ValueName, "", RegistryValueOptions.DoNotExpandEnvironmentNames) as string ?? "";
        }

        public void Save(string repository)
        {
            string selected = RepositoryValidator.Validate(repository);
            using var key = Registry.CurrentUser.CreateSubKey(keyPath);
            if (key == null) throw new IOException("Windows user preferences could not be opened.");
            key.SetValue(ValueName, selected, RegistryValueKind.String);
        }

        public int LoadViewerPort()
        {
            using var key = Registry.CurrentUser.OpenSubKey(keyPath);
            object? value = key?.GetValue("ViewerPort");
            return value is int port && port >= 1024 && port <= 65535
                ? port : SplattingTools.DefaultViewerPort;
        }

        public void SaveViewerPort(int port)
        {
            if (port < 1024 || port > 65535)
                throw new ArgumentOutOfRangeException(nameof(port), "Viewer Port must be from 1024 to 65535.");
            using var key = Registry.CurrentUser.CreateSubKey(keyPath);
            if (key == null) throw new IOException("Windows user preferences could not be opened.");
            key.SetValue("ViewerPort", port, RegistryValueKind.DWord);
        }
    }
}
