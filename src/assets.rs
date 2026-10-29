//! JAKA model assets (URDFs and meshes) for simulators and viewers.
//!
//! They are not in the crate or the repository. [`download_jaka_assets`]
//! installs the release archive into the shared asset directory,
//! `roplat_world::assets_dir()`, where a model's
//! [`RobotDescription`](roplat_behavior::RobotDescription) paths resolve.
//! Driving a real JAKA controller needs none of them.

use std::fs;
use std::io::{self, Cursor};
use std::path::{Path, PathBuf};

use reqwest::blocking::Client;
use roplat_behavior::{RobotException, RobotResult};

/// The release archive: `jaka/` with the URDFs and their meshes.
pub const JAKA_ASSETS_RELEASE_URL: &str =
    "https://github.com/Robot-Exp-Platform/libjaka-rs/releases/download/v0.1.15/jaka.zip";

/// Bump this when release assets change in an incompatible way.
pub const JAKA_ASSETS_VERSION: &str = "libjaka-assets-v0.1.15";

const JAKA_MARKER_FILE: &str = ".libjaka_assets_manifest";

/// The shared asset directory, `roplat_world::assets_dir()`.
pub fn roplat_assets_dir() -> Option<PathBuf> {
    roplat_world::assets_dir()
}

/// Whether the assets of [`JAKA_ASSETS_VERSION`] are installed.
pub fn is_jaka_assets_ready() -> bool {
    roplat_assets_dir().is_some_and(|assets| is_installed(&assets))
}

/// Downloads the release archive and installs it into the shared asset
/// directory, unless that version is installed already. Returns the
/// directory.
pub fn download_jaka_assets() -> RobotResult<PathBuf> {
    let assets = roplat_assets_dir().ok_or_else(|| {
        RobotException::UnprocessableInstructionError(
            "failed to determine user data directory".to_string(),
        )
    })?;
    if is_installed(&assets) {
        return Ok(assets);
    }
    install_jaka_archive(&download(JAKA_ASSETS_RELEASE_URL)?, &assets)?;
    Ok(assets)
}

/// Unpacks a release archive into `assets` and records its version.
pub fn install_jaka_archive(archive: &[u8], assets: &Path) -> RobotResult<()> {
    let mut zip = zip::ZipArchive::new(Cursor::new(archive))
        .map_err(zip_error("failed to read assets archive"))?;
    fs::create_dir_all(assets).map_err(io_error("failed to create assets directory"))?;
    // `extract` refuses entries that would land outside `assets`.
    zip.extract(assets)
        .map_err(zip_error("failed to extract assets archive"))?;
    fs::write(marker_path(assets), JAKA_ASSETS_VERSION)
        .map_err(io_error("failed to write marker"))
}

fn is_installed(assets: &Path) -> bool {
    let marker_ok = fs::read_to_string(marker_path(assets))
        .is_ok_and(|content| content.trim() == JAKA_ASSETS_VERSION);
    marker_ok && contains_extension_recursive(&assets.join("jaka"), "urdf")
}

/// Next to the asset directory, as earlier versions wrote it.
fn marker_path(assets: &Path) -> PathBuf {
    assets
        .parent()
        .unwrap_or(assets)
        .join(JAKA_MARKER_FILE)
}

fn download(url: &str) -> RobotResult<Vec<u8>> {
    let client = Client::builder()
        .build()
        .map_err(reqwest_error("failed to build HTTP client"))?;
    let response = client
        .get(url)
        .send()
        .map_err(reqwest_error("failed to request assets archive"))?
        .error_for_status()
        .map_err(reqwest_error("assets archive server returned error status"))?;
    let bytes = response
        .bytes()
        .map_err(reqwest_error("failed to read assets archive body"))?;
    Ok(bytes.to_vec())
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
                .is_some_and(|e| e.eq_ignore_ascii_case(ext))
            {
                return true;
            }
        }
    }
    false
}

fn io_error(context: &'static str) -> impl Fn(io::Error) -> RobotException {
    move |e| RobotException::UnprocessableInstructionError(format!("{context}: {e}"))
}

fn zip_error(context: &'static str) -> impl Fn(zip::result::ZipError) -> RobotException {
    move |e| RobotException::UnprocessableInstructionError(format!("{context}: {e}"))
}

fn reqwest_error(context: &'static str) -> impl Fn(reqwest::Error) -> RobotException {
    move |e| RobotException::UnprocessableInstructionError(format!("{context}: {e}"))
}

#[cfg(test)]
mod tests {
    use std::io::Write;

    use super::*;

    /// An archive laid out as the release one: entries under `jaka/`.
    fn archive() -> Vec<u8> {
        let mut zip = zip::ZipWriter::new(Cursor::new(Vec::new()));
        let options = zip::write::SimpleFileOptions::default();
        zip.add_directory("jaka/", options).unwrap();
        zip.start_file("jaka/jaka_zu3.urdf", options).unwrap();
        zip.write_all(b"<robot name=\"jaka_zu3\"/>").unwrap();
        zip.start_file("jaka/meshes/base.stl", options).unwrap();
        zip.write_all(b"solid").unwrap();
        zip.finish().unwrap().into_inner()
    }

    #[test]
    fn the_release_archive_installs_under_jaka() {
        let root = std::env::temp_dir().join(format!("libjaka-assets-{}", std::process::id()));
        let assets = root.join("assets");
        let _ = fs::remove_dir_all(&root);

        assert!(!is_installed(&assets));
        install_jaka_archive(&archive(), &assets).unwrap();
        assert!(assets.join("jaka/meshes/base.stl").is_file());
        assert_eq!(
            fs::read_to_string(assets.join("jaka/jaka_zu3.urdf")).unwrap(),
            "<robot name=\"jaka_zu3\"/>"
        );
        assert!(is_installed(&assets));

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_damaged_archive_is_an_error() {
        let assets = std::env::temp_dir().join("libjaka-assets-damaged");
        assert!(install_jaka_archive(b"not a zip", &assets).is_err());
    }
}
