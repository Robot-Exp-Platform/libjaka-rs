use std::env;
use std::fs::{self, File};
use std::io::{self, Write};
use std::path::{Path, PathBuf};

use flate2::read::GzDecoder;
use reqwest::blocking::Client;
use robot_behavior::{RobotException, RobotResult};

/// Replace this URL with your real GitHub Release artifact after publishing.
pub const JAKA_ASSETS_RELEASE_URL: &str =
    "https://github.com/Robot-Exp-Platform/libjaka-rs/releases/download/v0.1.15/jaka.zip";

/// Bump this when release assets change in an incompatible way.
pub const JAKA_ASSETS_VERSION: &str = "libjaka-assets-v0.1.15";

const JAKA_MARKER_FILE: &str = ".libjaka_assets_manifest";

/// Return the target assets root following build.rs convention: <data>/roplat/assets.
pub fn roplat_assets_dir() -> Option<PathBuf> {
    roplat_data_target_dir().map(|root| root.join("roplat").join("assets"))
}

/// Check whether JAKA assets are present and match the expected local marker.
pub fn is_jaka_assets_ready() -> bool {
    let Some(assets_root) = roplat_assets_dir() else {
        return false;
    };

    let roplat_root = match assets_root.parent() {
        Some(p) => p,
        None => return false,
    };

    let marker_path = roplat_root.join(JAKA_MARKER_FILE);
    let marker_ok = fs::read_to_string(marker_path)
        .map(|content| content.trim() == JAKA_ASSETS_VERSION)
        .unwrap_or(false);

    let jaka_assets = assets_root.join("jaka");
    let has_urdf = jaka_assets.exists() && contains_extension_recursive(&jaka_assets, "urdf");

    marker_ok && has_urdf
}

/// Explicitly download and install JAKA assets from release archive.
///
/// Expected archive layout:
/// - either contains top-level `assets/`
/// - or contains `<something>/assets/`
pub fn download_jaka_assets() -> RobotResult<PathBuf> {
    let data_root = roplat_data_target_dir().ok_or_else(|| {
        RobotException::UnprocessableInstructionError(
            "failed to determine user data directory".to_string(),
        )
    })?;

    let roplat_root = data_root.join("roplat");
    let target_assets = roplat_root.join("assets");
    let marker_path = roplat_root.join(JAKA_MARKER_FILE);

    fs::create_dir_all(&roplat_root).map_err(io_error("failed to create roplat root"))?;
    fs::create_dir_all(&target_assets).map_err(io_error("failed to create assets root"))?;

    if is_jaka_assets_ready() {
        return Ok(target_assets);
    }

    let archive_path = roplat_root.join("libjaka-assets.tar.gz");
    let unpack_root = roplat_root.join(".libjaka-assets-unpack");

    if unpack_root.exists() {
        fs::remove_dir_all(&unpack_root).map_err(io_error("failed to clean old unpack dir"))?;
    }
    fs::create_dir_all(&unpack_root).map_err(io_error("failed to create unpack dir"))?;

    download_to_file(JAKA_ASSETS_RELEASE_URL, &archive_path)?;
    extract_tar_gz(&archive_path, &unpack_root)?;

    let extracted_assets = find_assets_dir(&unpack_root).ok_or_else(|| {
        RobotException::UnprocessableInstructionError(
            "downloaded archive does not contain an assets directory".to_string(),
        )
    })?;

    copy_dir_recursive(&extracted_assets, &target_assets)?;
    fs::write(&marker_path, JAKA_ASSETS_VERSION).map_err(io_error("failed to write marker"))?;

    let _ = fs::remove_file(&archive_path);
    let _ = fs::remove_dir_all(&unpack_root);

    Ok(target_assets)
}

fn download_to_file(url: &str, dest: &Path) -> RobotResult<()> {
    let client = Client::builder()
        .build()
        .map_err(reqwest_error("failed to build HTTP client"))?;

    let response = client
        .get(url)
        .send()
        .map_err(reqwest_error("failed to request assets archive"))?;

    let response = response
        .error_for_status()
        .map_err(reqwest_error("assets archive server returned error status"))?;

    let bytes = response
        .bytes()
        .map_err(reqwest_error("failed to read assets archive body"))?;

    let mut file = File::create(dest).map_err(io_error("failed to create archive file"))?;
    file.write_all(&bytes)
        .map_err(io_error("failed to write archive file"))?;
    Ok(())
}

fn extract_tar_gz(archive_path: &Path, out_dir: &Path) -> RobotResult<()> {
    let tar_gz = File::open(archive_path).map_err(io_error("failed to open archive file"))?;
    let dec = GzDecoder::new(tar_gz);
    let mut archive = tar::Archive::new(dec);
    archive
        .unpack(out_dir)
        .map_err(io_error("failed to extract assets archive"))
}

fn find_assets_dir(root: &Path) -> Option<PathBuf> {
    if root.file_name().and_then(|n| n.to_str()) == Some("assets") {
        return Some(root.to_path_buf());
    }
    let direct = root.join("assets");
    if direct.is_dir() {
        return Some(direct);
    }

    let entries = fs::read_dir(root).ok()?;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir()
            && let Some(found) = find_assets_dir(&path)
        {
            return Some(found);
        }
    }
    None
}

fn copy_dir_recursive(src: &Path, dst: &Path) -> RobotResult<()> {
    if !src.is_dir() {
        return Err(RobotException::UnprocessableInstructionError(format!(
            "source is not a directory: {}",
            src.display()
        )));
    }

    fs::create_dir_all(dst).map_err(io_error("failed to create target directory"))?;
    for entry in fs::read_dir(src).map_err(io_error("failed to read source directory"))? {
        let entry = entry.map_err(io_error("failed to iterate source directory"))?;
        let src_path = entry.path();
        let dst_path = dst.join(entry.file_name());

        if src_path.is_dir() {
            copy_dir_recursive(&src_path, &dst_path)?;
        } else if src_path.is_file() {
            if let Some(parent) = dst_path.parent() {
                fs::create_dir_all(parent)
                    .map_err(io_error("failed to create target parent directory"))?;
            }
            fs::copy(&src_path, &dst_path).map_err(io_error("failed to copy asset file"))?;
        }
    }
    Ok(())
}

fn contains_extension_recursive(root: &Path, ext: &str) -> bool {
    let mut stack = vec![root.to_path_buf()];
    while let Some(path) = stack.pop() {
        let Ok(entries) = fs::read_dir(&path) else {
            continue;
        };
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() {
                stack.push(p);
            } else if p
                .extension()
                .and_then(|e| e.to_str())
                .map(|e| e.eq_ignore_ascii_case(ext))
                .unwrap_or(false)
            {
                return true;
            }
        }
    }
    false
}

fn roplat_data_target_dir() -> Option<PathBuf> {
    #[cfg(target_os = "windows")]
    {
        env::var_os("LOCALAPPDATA")
            .map(PathBuf::from)
            .or_else(|| env::var_os("USERPROFILE").map(PathBuf::from))
    }

    #[cfg(target_os = "macos")]
    {
        env::var_os("HOME")
            .map(PathBuf::from)
            .map(|home| home.join("Library").join("Application Support"))
    }

    #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
    {
        env::var_os("HOME")
            .map(PathBuf::from)
            .map(|home| home.join(".local").join("share"))
    }
}

fn io_error(context: &'static str) -> impl Fn(io::Error) -> RobotException {
    move |e| RobotException::UnprocessableInstructionError(format!("{context}: {e}"))
}

fn reqwest_error(context: &'static str) -> impl Fn(reqwest::Error) -> RobotException {
    move |e| RobotException::UnprocessableInstructionError(format!("{context}: {e}"))
}
