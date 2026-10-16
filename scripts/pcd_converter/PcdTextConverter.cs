using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Globalization;
using System.IO;
using System.Text;

namespace PcdConverter
{

public static class PcdTextConverter
{
    // Streams the input; it never loads the full room map into RAM.
    public static long Convert(string source, string destination, bool overwrite = false,
        CancellationToken cancellationToken = default, IProgress<long>? progress = null)
    {
        source = Path.GetFullPath(source);
        destination = Path.GetFullPath(destination);
        if (string.Equals(source, destination, StringComparison.OrdinalIgnoreCase))
            throw new IOException("Input and output must be different files.");
        if (!source.EndsWith(".pcd", StringComparison.OrdinalIgnoreCase) ||
            !destination.EndsWith(".txt", StringComparison.OrdinalIgnoreCase))
            throw new InvalidDataException("Only PCD to TXT conversion is currently supported.");
        if (File.Exists(destination) && !overwrite)
            throw new IOException($"The output file already exists: {destination}");

        using var reader = new StreamReader(source, Encoding.UTF8, true, 65536);
        var header = new Dictionary<string, string[]>(StringComparer.OrdinalIgnoreCase);
        bool foundData = false;
        for (int lineNumber = 0; lineNumber < 1000; lineNumber++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            string? line = reader.ReadLine();
            if (line is null) break;
            string[] tokens = Split(line);
            if (tokens.Length == 0 || tokens[0].StartsWith("#", StringComparison.Ordinal)) continue;
            string key = tokens[0];
            if (key.Equals("DATA", StringComparison.OrdinalIgnoreCase))
            {
                if (tokens.Length != 2 || !tokens[1].Equals("ascii", StringComparison.OrdinalIgnoreCase))
                    throw new InvalidDataException("Only DATA ascii PCD files are supported. Binary/compressed PCD is not supported yet.");
                foundData = true;
                break;
            }
            if (header.ContainsKey(key))
                throw new InvalidDataException($"Duplicate PCD header: {key}.");
            header.Add(key, tokens.Skip(1).ToArray());
        }
        if (!foundData) throw new InvalidDataException("Missing PCD DATA ascii header.");
        string[] fields = Required(header, "FIELDS");
        string[] sizes = Required(header, "SIZE");
        string[] types = Required(header, "TYPE");
        string[] counts = header.TryGetValue("COUNT", out var declaredCounts)
            ? declaredCounts : Enumerable.Repeat("1", fields.Length).ToArray();
        if (fields.Length == 0 || sizes.Length != fields.Length || types.Length != fields.Length || counts.Length != fields.Length)
            throw new InvalidDataException("PCD FIELDS, SIZE, TYPE and COUNT must have matching lengths.");

        int columns = 0;
        var offsets = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < fields.Length; i++)
        {
            int count = PositiveInt(counts[i], "COUNT");
            int size = PositiveInt(sizes[i], "SIZE");
            string type = types[i].ToUpperInvariant();
            if ((type != "F" && type != "I" && type != "U") ||
                (type == "F" ? size != 4 && size != 8 : size != 1 && size != 2 && size != 4 && size != 8))
                throw new InvalidDataException("Unsupported PCD scalar type or size.");
            if (offsets.ContainsKey(fields[i]))
                throw new InvalidDataException($"Duplicate PCD field: {fields[i]}.");
            offsets.Add(fields[i], columns);
            if (new[] { "x", "y", "z", "intensity" }.Contains(fields[i], StringComparer.OrdinalIgnoreCase) && count != 1)
                throw new InvalidDataException("XYZ and intensity must be scalar fields.");
            columns = checked(columns + count);
        }
        foreach (string field in new[] { "x", "y", "z", "intensity" })
            if (!offsets.ContainsKey(field))
                throw new InvalidDataException($"PCD is missing the {field} field. No coordinates or intensity will be invented.");

        long width = NonNegativeLong(Required(header, "WIDTH"), "WIDTH");
        long height = NonNegativeLong(Required(header, "HEIGHT"), "HEIGHT");
        long expected = header.TryGetValue("POINTS", out var points)
            ? NonNegativeLong(points, "POINTS") : checked(width * height);
        if (expected == 0 || expected != checked(width * height))
            throw new InvalidDataException("PCD must contain points and POINTS must equal WIDTH times HEIGHT.");
        int[] selected = { offsets["x"], offsets["y"], offsets["z"], offsets["intensity"] };

        string temporary = destination + "." + Guid.NewGuid().ToString("N") + ".tmp";
        long written = 0;
        try
        {
            using (var file = new FileStream(temporary, FileMode.CreateNew, FileAccess.Write, FileShare.None, 65536))
            using (var writer = new StreamWriter(file, new UTF8Encoding(false), 65536))
            {
                while (reader.ReadLine() is { } line)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    string[] tokens = Split(line);
                    if (tokens.Length == 0 || tokens[0].StartsWith("#", StringComparison.Ordinal)) continue;
                    if (tokens.Length != columns)
                        throw new InvalidDataException($"Point {written + 1}: expected {columns} values, received {tokens.Length}.");
                    for (int i = 0; i < selected.Length; i++)
                    {
                        if (!double.TryParse(tokens[selected[i]], NumberStyles.Float, CultureInfo.InvariantCulture, out double number) ||
                            double.IsNaN(number) || double.IsInfinity(number) || (i == 3 && (number < 0 || number > 255)))
                            throw new InvalidDataException($"Point {written + 1}: XYZ must be finite and intensity must be between 0 and 255.");
                    }
                    if (++written > expected) throw new InvalidDataException("PCD contains more points than declared.");
                    writer.WriteLine(string.Join(" ", selected.Select(index => tokens[index])));
                    if (written % 10000 == 0) progress?.Report(written);
                }
                if (written != expected)
                    throw new InvalidDataException($"PCD declares {expected} points but contains {written}.");
            }
            cancellationToken.ThrowIfCancellationRequested();
            if (overwrite && File.Exists(destination)) File.Replace(temporary, destination, null);
            else File.Move(temporary, destination);
            progress?.Report(written);
            return written;
        }
        finally
        {
            if (File.Exists(temporary)) File.Delete(temporary);
        }
    }

    private static string[] Split(string line) => line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
    private static string[] Required(Dictionary<string, string[]> header, string key) =>
        header.TryGetValue(key, out var value) ? value : throw new InvalidDataException($"Missing PCD {key} header.");
    private static int PositiveInt(string value, string name) =>
        int.TryParse(value, NumberStyles.None, CultureInfo.InvariantCulture, out int result) && result > 0
            ? result : throw new InvalidDataException($"Invalid PCD {name}.");
    private static long NonNegativeLong(string[] value, string name) =>
        value.Length == 1 && long.TryParse(value[0], NumberStyles.None, CultureInfo.InvariantCulture, out long result) && result >= 0
            ? result : throw new InvalidDataException($"Invalid PCD {name}.");
}
}
