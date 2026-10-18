using System;
using System.IO;
using Microsoft.Win32;

namespace RawDataSender
{
    public sealed class SenderPreferences
    {
        public const string DefaultKeyPath = @"Software\VISTA\RawDataSender";
        private readonly string keyPath;
        public SenderPreferences(string keyPath = DefaultKeyPath)
        {
            if (string.IsNullOrWhiteSpace(keyPath) || (keyPath != DefaultKeyPath && !keyPath.StartsWith(DefaultKeyPath + "\\", StringComparison.Ordinal)))
                throw new ArgumentException("Preferences must belong to RawDataSender.");
            this.keyPath = keyPath;
        }
        public string LoadFile()
        {
            using var key = Registry.CurrentUser.OpenSubKey(keyPath);
            return key?.GetValue("LastFile", "", RegistryValueOptions.DoNotExpandEnvironmentNames) as string ?? "";
        }
        public string LoadDirectory()
        {
            using var key = Registry.CurrentUser.OpenSubKey(keyPath);
            return key?.GetValue("LastDirectory", "", RegistryValueOptions.DoNotExpandEnvironmentNames) as string ?? "";
        }
        public string LoadPort()
        {
            using var key = Registry.CurrentUser.OpenSubKey(keyPath);
            return key?.GetValue("LastPort", "", RegistryValueOptions.DoNotExpandEnvironmentNames) as string ?? "";
        }
        public int LoadBaud()
        {
            using var key = Registry.CurrentUser.OpenSubKey(keyPath);
            object? value = key?.GetValue("BaudRate");
            return value is int baud && baud >= 300 && baud <= 12000000 ? baud : 4000000;
        }
        public void SaveFile(string file)
        {
            string path = Path.GetFullPath(file);
            if (!File.Exists(path)) throw new FileNotFoundException("The selected file no longer exists.", path);
            using var key = Registry.CurrentUser.CreateSubKey(keyPath);
            if (key == null) throw new IOException("Windows user preferences could not be opened.");
            key.SetValue("LastFile", path, RegistryValueKind.String);
            key.SetValue("LastDirectory", Path.GetDirectoryName(path) ?? "", RegistryValueKind.String);
        }
        public void SaveConnection(string port, int baud)
        {
            using var key = Registry.CurrentUser.CreateSubKey(keyPath);
            if (key == null) throw new IOException("Windows user preferences could not be opened.");
            key.SetValue("LastPort", port, RegistryValueKind.String);
            key.SetValue("BaudRate", baud, RegistryValueKind.DWord);
        }
    }
}
