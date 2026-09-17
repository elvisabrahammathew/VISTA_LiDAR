//! Selects and links the bundled librealsense runtime for the Cargo target.

use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

fn main() {
    println!("cargo:rustc-check-cfg=cfg(vista_realsense)");
    println!("cargo:rerun-if-changed=third_party/realsense");

    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("manifest dir"));
    let target_os = env::var("CARGO_CFG_TARGET_OS").expect("target OS");
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").expect("target architecture");
    let host = env::var("HOST").expect("Cargo host triple");
    let target = env::var("TARGET").expect("Cargo target triple");

    match target_os.as_str() {
        "windows" => configure_windows(&manifest_dir, &host, &target),
        "linux" => configure_linux(&manifest_dir, &target_arch, &host, &target),
        _ => println!("cargo:warning=librealsense is not configured for {target_os}/{target_arch}"),
    }
}

/// Uses bundled Windows binaries, or builds the SDK source when they are absent.
fn configure_windows(manifest_dir: &Path, host: &str, target: &str) {
    let sdk_dir = manifest_dir.join("third_party/realsense/window");
    let import_library = sdk_dir.join("realsense2.lib");
    let runtime_library = sdk_dir.join("realsense2.dll");

    if import_library.is_file() && runtime_library.is_file() {
        enable_windows_runtime(&import_library, &runtime_library);
        return;
    }

    if host == target {
        if let Some(source_dir) = find_realsense_source(manifest_dir) {
            let build_dir = build_realsense_source(&source_dir);
            let built_import = find_file(&build_dir, "realsense2.lib");
            let built_runtime = find_file(&build_dir, "realsense2.dll");
            if let (Some(import), Some(runtime)) = (built_import, built_runtime) {
                enable_windows_runtime(&import, &runtime);
                return;
            }
            panic!(
                "librealsense build completed but realsense2.lib/realsense2.dll were not found below {}",
                build_dir.display()
            );
        }
    }

    println!(
        "cargo:warning=RealSense Windows backend disabled: no prebuilt runtime or native SDK source build is available"
    );
}

fn enable_windows_runtime(import_library: &Path, runtime_library: &Path) {
    let library_dir = import_library.parent().expect("Windows library directory");
    println!("cargo:rustc-link-search=native={}", library_dir.display());
    println!("cargo:rustc-link-lib=dylib=realsense2");
    println!("cargo:rustc-cfg=vista_realsense");
    copy_runtime_library(runtime_library, "realsense2.dll");
}

/// Uses a bundled Linux library, or builds the SDK source on native Linux.
fn configure_linux(manifest_dir: &Path, target_arch: &str, host: &str, target: &str) {
    let directory_name = match target_arch {
        "x86_64" => "linux_x8664",
        "aarch64" => "linux_arm64",
        _ => {
            println!(
                "cargo:warning=RealSense Linux backend disabled for architecture {target_arch}"
            );
            return;
        }
    };
    let sdk_dir = manifest_dir
        .join("third_party/realsense")
        .join(directory_name);
    let shared_library = sdk_dir.join("librealsense2.so");

    if shared_library.is_file() {
        enable_linux_runtime(&shared_library);
        return;
    }

    if host == target {
        if let Some(source_dir) = find_realsense_source(manifest_dir) {
            let build_dir = build_realsense_source(&source_dir);
            if let Some(runtime) = find_file(&build_dir, "librealsense2.so") {
                enable_linux_runtime(&runtime);
                return;
            }
            panic!(
                "librealsense build completed but librealsense2.so was not found below {}",
                build_dir.display()
            );
        }
    }

    println!(
        "cargo:warning=RealSense Linux backend disabled: no prebuilt library exists for {target_arch}, and source builds require a native {target} host"
    );
}

fn enable_linux_runtime(shared_library: &Path) {
    let library_dir = shared_library.parent().expect("Linux library directory");
    println!("cargo:rustc-link-search=native={}", library_dir.display());
    println!("cargo:rustc-link-lib=dylib=realsense2");
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
    println!("cargo:rustc-cfg=vista_realsense");
    copy_runtime_library(shared_library, "librealsense2.so");
}

/// Finds the Rust-local SDK source first and the current C++ copy second.
fn find_realsense_source(manifest_dir: &Path) -> Option<PathBuf> {
    let candidates = [
        manifest_dir.join("third_party/realsense/librealsense-2.50.0"),
        manifest_dir.join("../embedded_C/third_party/realsense/librealsense-2.50.0"),
    ];
    candidates
        .into_iter()
        .find(|path| path.join("CMakeLists.txt").is_file())
}

/// Builds a generated shared runtime under Cargo's target directory.
fn build_realsense_source(source_dir: &Path) -> PathBuf {
    let output_dir = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo OUT_DIR"));
    let build_dir = output_dir.join("librealsense-build");
    fs::create_dir_all(&build_dir).expect("create librealsense build directory");
    println!(
        "cargo:rerun-if-changed={}",
        source_dir.join("CMakeLists.txt").display()
    );

    run_cmake(
        Command::new("cmake")
            .arg("-S")
            .arg(source_dir)
            .arg("-B")
            .arg(&build_dir)
            .args([
                "-DCMAKE_BUILD_TYPE=Release",
                "-DBUILD_SHARED_LIBS=ON",
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
            ]),
        "configure librealsense",
    );
    run_cmake(
        Command::new("cmake").arg("--build").arg(&build_dir).args([
            "--config",
            "Release",
            "--target",
            "realsense2",
            "--parallel",
        ]),
        "build librealsense",
    );
    build_dir
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

/// Places a native runtime where both Cargo binaries and test executables can load it.
fn copy_runtime_library(runtime_library: &Path, file_name: &str) {
    let output_dir = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo OUT_DIR"));
    let Some(profile_dir) = output_dir.ancestors().nth(3) else {
        println!("cargo:warning=could not determine Cargo profile directory for {file_name}");
        return;
    };

    for destination_dir in [profile_dir.to_path_buf(), profile_dir.join("deps")] {
        if let Err(error) = fs::create_dir_all(&destination_dir)
            .and_then(|()| fs::copy(runtime_library, destination_dir.join(file_name)).map(|_| ()))
        {
            panic!(
                "failed to copy {} to {}: {error}",
                runtime_library.display(),
                destination_dir.display()
            );
        }
    }
}
