using System;
using System.IO;

namespace PcdConverter
{

public static class RepositoryValidator
{
    public static string Validate(string repository)
    {
        if (string.IsNullOrWhiteSpace(repository))
            throw new InvalidDataException("Select a Gaussian Splatting repository first.");
        repository = Path.GetFullPath(repository.Trim().Trim('"'));
        if (!Directory.Exists(repository))
            throw new InvalidDataException($"The selected repository folder does not exist:\n\n{repository}");
        string[] required = { "package.json", "package-lock.json", "pyproject.toml", "uv.lock",
            Path.Combine("python", "lidar_splat_preprocess", "cli.py") };
        foreach (string file in required)
            if (!File.Exists(Path.Combine(repository, file)))
                throw new InvalidDataException($"The repository is missing {file}.\n\n" +
                    $"Selected folder: {repository}\n\nChoose the root folder of the lidar_surface_splatting repository.");
        return repository;
    }
}
}
