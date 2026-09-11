//! Selects and links the bundled librealsense runtime for the Cargo target.

use std::{
    env, fs,
    path::{Path, PathBuf},
};

fn main() {
    println!("cargo:rustc-check-cfg=cfg(vista_realsense)");
    println!("cargo:rerun-if-changed=third_party/realsense");

    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("manifest dir"));
    let target_os = env::var("CARGO_CFG_TARGET_OS").expect("target OS");
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").expect("target architecture");

    match target_os.as_str() {
        "windows" => configure_windows(&manifest_dir),
        "linux" => configure_linux(&manifest_dir, &target_arch),
        _ => println!("cargo:warning=librealsense is not configured for {target_os}/{target_arch}"),
    }
}

/// Links the Windows import library and copies the runtime DLL beside Cargo binaries.
fn configure_windows(manifest_dir: &Path) {
    let sdk_dir = manifest_dir.join("third_party/realsense/window");
    let import_library = sdk_dir.join("realsense2.lib");
    let runtime_library = sdk_dir.join("realsense2.dll");

    if !import_library.is_file() || !runtime_library.is_file() {
        println!(
            "cargo:warning=RealSense Windows backend disabled: expected {} and {}",
            import_library.display(),
            runtime_library.display()
        );
        return;
    }

    println!("cargo:rustc-link-search=native={}", sdk_dir.display());
    println!("cargo:rustc-link-lib=dylib=realsense2");
    println!("cargo:rustc-cfg=vista_realsense");
    copy_runtime_library(&runtime_library, "realsense2.dll");
}

/// Links a bundled Linux shared library when one exists for the current CPU.
fn configure_linux(manifest_dir: &Path, target_arch: &str) {
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

    if !shared_library.is_file() {
        println!(
            "cargo:warning=RealSense Linux backend disabled: {} is missing",
            shared_library.display()
        );
        return;
    }

    println!("cargo:rustc-link-search=native={}", sdk_dir.display());
    println!("cargo:rustc-link-lib=dylib=realsense2");
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
    println!("cargo:rustc-cfg=vista_realsense");
    copy_runtime_library(&shared_library, "librealsense2.so");
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
