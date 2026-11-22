//! Keep one copy of the licence beside every `Cargo.toml`.
//!
//! Cargo packages only files under a package's own directory, so a licence kept
//! at the workspace root never reaches a published tarball. That is the whole
//! reason the copies exist: 0BSD asks for nothing but its own text, and a crate
//! published without that text is published without its licence.
//!
//! The root `LICENSE` is the source of truth. `cargo xtask licences` writes it
//! next to every manifest — the published crates and the unpublished firmware
//! and tooling alike, so a reader finds the terms wherever they are — and
//! `--check` fails if any copy has drifted. CI runs the check.

use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use crate::repo_root;

/// The licence file's name, at the root and in every crate.
const NAME: &str = "LICENSE";

pub fn run(check: bool) -> Result<(), Box<dyn Error>> {
    let root = repo_root();
    let source = root.join(NAME);
    let text =
        fs::read_to_string(&source).map_err(|e| format!("reading {}: {e}", source.display()))?;

    let mut manifests = Vec::new();
    manifests_with(&root, &mut manifests)?;

    let mut stale = Vec::new();
    for manifest in &manifests {
        let dir = manifest.parent().expect("a manifest has a parent");
        let dest = dir.join(NAME);
        if dest == source || fs::read_to_string(&dest).is_ok_and(|t| t == text) {
            continue;
        }
        if check {
            stale.push(dest);
        } else {
            fs::write(&dest, &text)?;
            println!("wrote {}", dest.display());
        }
    }

    if check && !stale.is_empty() {
        for dest in &stale {
            eprintln!("{} is out of date with the root {NAME}", dest.display());
        }
        return Err(format!("{} crate(s) need `cargo xtask licences`", stale.len()).into());
    }
    if check {
        println!("licence copies are up to date");
    }
    Ok(())
}

/// Every `Cargo.toml` under `dir`, skipping build output and the git database.
fn manifests_with(dir: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let name = entry.file_name();
        if entry.file_type()?.is_dir() {
            if name != "target" && name != ".git" {
                manifests_with(&entry.path(), out)?;
            }
        } else if name == "Cargo.toml" {
            out.push(entry.path());
        }
    }
    Ok(())
}
