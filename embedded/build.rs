//! Builds and statically links librealsense for the native Cargo target.

use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

fn main() {
    println!("cargo:rustc-check-cfg=cfg(vista_realsense)");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=../third_party/librealsense/CMakeLists.txt");
    println!("cargo:rerun-if-changed=GrafanaSecret.txt");

    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("manifest dir"));
    copy_optional_grafana_secret(&manifest_dir);

    let target_os = env::var("CARGO_CFG_TARGET_OS").expect("target OS");
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").expect("target architecture");
    let host = env::var("HOST").expect("Cargo host triple");
    let target = env::var("TARGET").expect("Cargo target triple");

    if !matches!(
        (target_os.as_str(), target_arch.as_str()),
        ("windows", "x86_64") | ("linux", "x86_64") | ("linux", "aarch64")
    ) {
        println!("cargo:warning=RealSense is unsupported for target {target_os}/{target_arch}");
        return;
    }

    if host != target {
        println!(
            "cargo:warning=RealSense backend disabled: static librealsense must be built natively for {target}"
        );
        return;
    }

    let source_dir = manifest_dir.join("../third_party/librealsense");
    if !source_dir.join("CMakeLists.txt").is_file() {
        println!(
            "cargo:warning=RealSense backend disabled: initialize the repository-level submodule with `git submodule update --init --recursive`"
        );
        return;
    }

    let output_dir = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo OUT_DIR"));
    let build_dir = output_dir.join(format!("librealsense-static-{target_os}-{target_arch}"));
    build_static_librealsense(&source_dir, &build_dir, &target_os, &target_arch);
    link_static_librealsense(&build_dir, &target_os, &target_arch);
    println!("cargo:rustc-cfg=vista_realsense");
}

fn build_static_librealsense(
    source_dir: &Path,
    build_dir: &Path,
    target_os: &str,
    target_arch: &str,
) {
    fs::create_dir_all(build_dir).expect("create librealsense build directory");
    let rsusb = if target_os == "linux" && target_arch == "aarch64" {
        "ON"
    } else {
        "OFF"
    };

    run_cmake(
        Command::new("cmake")
            .arg("-S")
            .arg(source_dir)
            .arg("-B")
            .arg(build_dir)
            .args([
                "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
                "-DCMAKE_BUILD_TYPE=Release",
                "-DBUILD_SHARED_LIBS=OFF",
                "-DBUILD_WITH_STATIC_CRT=OFF",
                "-DBUILD_EXAMPLES=OFF",
                "-DBUILD_GRAPHICAL_EXAMPLES=OFF",
                "-DBUILD_TOOLS=OFF",
                "-DBUILD_UNIT_TESTS=OFF",
                "-DBUILD_INTERNAL_UNIT_TESTS=OFF",
                "-DBUILD_LEGACY_LIVE_TEST=OFF",
                "-DBUILD_WITH_TM2=OFF",
                "-DIMPORT_DEPTH_CAM_FW=OFF",
                "-DCHECK_FOR_UPDATES=OFF",
                "-DBUILD_GLSL_EXTENSIONS=OFF",
                "-DBUILD_NETWORK_DEVICE=OFF",
                "-DBUILD_WITH_CUDA=OFF",
                "-DBUILD_WITH_OPENMP=OFF",
                "-DENABLE_CCACHE=OFF",
            ])
            .arg(format!("-DFORCE_RSUSB_BACKEND={rsusb}")),
        "configure static librealsense",
    );
    run_cmake(
        Command::new("cmake").arg("--build").arg(build_dir).args([
            "--config",
            "Release",
            "--target",
            "realsense2",
            "--parallel",
        ]),
        "build static librealsense",
    );
}

fn link_static_librealsense(build_dir: &Path, target_os: &str, target_arch: &str) {
    match target_os {
        "windows" => {
            link_archive(build_dir, "realsense2.lib", "realsense2");
            link_archive(build_dir, "realsense-file.lib", "realsense-file");
            // Static librealsense uses Windows security-descriptor helpers.
            println!("cargo:rustc-link-lib=dylib=advapi32");
        }
        "linux" => {
            link_archive(build_dir, "librealsense2.a", "realsense2");
            link_archive(build_dir, "librealsense-file.a", "realsense-file");
            println!("cargo:rustc-link-lib=dylib=stdc++");
            println!("cargo:rustc-link-lib=dylib=usb-1.0");
            if target_arch == "x86_64" {
                println!("cargo:rustc-link-lib=dylib=udev");
            }
            for library in ["pthread", "dl", "rt", "m"] {
                println!("cargo:rustc-link-lib=dylib={library}");
            }
        }
        _ => unreachable!("unsupported librealsense target OS"),
    }
}

fn link_archive(build_dir: &Path, file_name: &str, link_name: &str) {
    let archive = find_file(build_dir, file_name).unwrap_or_else(|| {
        panic!(
            "librealsense build completed but {file_name} was not found below {}",
            build_dir.display()
        )
    });
    let library_dir = archive.parent().expect("static library directory");
    println!("cargo:rustc-link-search=native={}", library_dir.display());
    println!("cargo:rustc-link-lib=static={link_name}");
}

fn run_cmake(command: &mut Command, operation: &str) {
    let status = command.status().unwrap_or_else(|error| {
        panic!("could not run CMake to {operation}: {error}");
    });
    assert!(status.success(), "CMake failed to {operation}: {status}");
}

fn find_file(root: &Path, file_name: &str) -> Option<PathBuf> {
    let entries = fs::read_dir(root).ok()?;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.file_name().and_then(|value| value.to_str()) == Some(file_name) {
            return Some(path);
        }
        if path.is_dir() {
            if let Some(found) = find_file(&path, file_name) {
                return Some(found);
            }
        }
    }
    None
}

fn copy_file(source: &Path, destination: &Path) {
    let parent = destination.parent().expect("destination directory");
    fs::create_dir_all(parent).unwrap_or_else(|error| {
        panic!("failed to create {}: {error}", parent.display());
    });
    fs::copy(source, destination).unwrap_or_else(|error| {
        panic!(
            "failed to copy {} to {}: {error}",
            source.display(),
            destination.display()
        );
    });
}

/// Keeps the ignored local token beside Cargo binaries without embedding it.
fn copy_optional_grafana_secret(manifest_dir: &Path) {
    let source = manifest_dir.join("GrafanaSecret.txt");
    if !source.is_file() {
        return;
    }
    let output_dir = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo OUT_DIR"));
    let Some(profile_dir) = output_dir.ancestors().nth(3) else {
        println!("cargo:warning=could not locate Cargo profile directory for GrafanaSecret.txt");
        return;
    };
    copy_file(&source, &profile_dir.join("GrafanaSecret.txt"));
}
