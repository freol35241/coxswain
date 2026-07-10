//! Host tool: validate, compile+sign, inspect. Three subcommands, hand-rolled
//! argument handling.

use std::process::ExitCode;

const USAGE: &str = "usage:
  coxswain-manifest validate <manifest.toml>
  coxswain-manifest compile <manifest.toml> --key <seed.bin> -o <out.cxmanifest>
  coxswain-manifest inspect <blob> --pubkey <hex-or-file>
  coxswain-manifest pubkey --key <seed.bin>";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("validate") => cmd_validate(&args[1..]),
        Some("compile") => cmd_compile(&args[1..]),
        Some("inspect") => cmd_inspect(&args[1..]),
        Some("pubkey") => cmd_pubkey(&args[1..]),
        _ => Err(USAGE.to_string()),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("{message}");
            ExitCode::FAILURE
        }
    }
}

fn cmd_validate(args: &[String]) -> Result<(), String> {
    let [path] = args else {
        return Err(USAGE.to_string());
    };
    let source = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    coxswain_manifest::validate(&source).map_err(|e| e.to_string())?;
    println!("ok");
    Ok(())
}

fn cmd_compile(args: &[String]) -> Result<(), String> {
    let (mut manifest_path, mut key_path, mut out_path) = (None, None, None);
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--key" => key_path = Some(iter.next().ok_or(USAGE)?.clone()),
            "-o" => out_path = Some(iter.next().ok_or(USAGE)?.clone()),
            _ if manifest_path.is_none() => manifest_path = Some(arg.clone()),
            _ => return Err(USAGE.to_string()),
        }
    }
    let (Some(manifest_path), Some(key_path), Some(out_path)) = (manifest_path, key_path, out_path)
    else {
        return Err(USAGE.to_string());
    };

    let source =
        std::fs::read_to_string(&manifest_path).map_err(|e| format!("{manifest_path}: {e}"))?;
    let manifest = coxswain_manifest::compile(&source).map_err(|e| e.to_string())?;
    let seed = read_seed(&key_path)?;
    let blob = coxswain_manifest::write(&manifest, &seed);
    std::fs::write(&out_path, &blob).map_err(|e| format!("{out_path}: {e}"))?;
    println!("{}", hex(&coxswain_manifest::manifest_hash(&blob)));
    Ok(())
}

fn cmd_inspect(args: &[String]) -> Result<(), String> {
    let (mut blob_path, mut pubkey_arg) = (None, None);
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--pubkey" => pubkey_arg = Some(iter.next().ok_or(USAGE)?.clone()),
            _ if blob_path.is_none() => blob_path = Some(arg.clone()),
            _ => return Err(USAGE.to_string()),
        }
    }
    let (Some(blob_path), Some(pubkey_arg)) = (blob_path, pubkey_arg) else {
        return Err(USAGE.to_string());
    };

    let blob = std::fs::read(&blob_path).map_err(|e| format!("{blob_path}: {e}"))?;
    let pubkey = read_pubkey(&pubkey_arg)?;
    let manifest =
        coxswain_manifest::read(&blob, &pubkey).map_err(|e| format!("{blob_path}: {e}"))?;
    println!("vessel_id: {}", manifest.vessel_id.as_str());
    println!("name:      {}", manifest.name.as_str());
    println!("revision:  {}", manifest.revision);
    println!(
        "hash:      {}",
        hex(&coxswain_manifest::manifest_hash(&blob))
    );
    Ok(())
}

fn cmd_pubkey(args: &[String]) -> Result<(), String> {
    let [flag, key_path] = args else {
        return Err(USAGE.to_string());
    };
    if flag != "--key" {
        return Err(USAGE.to_string());
    }
    let seed = read_seed(key_path)?;
    println!("{}", hex(&coxswain_manifest::public_key(&seed)));
    Ok(())
}

/// A signing seed file: exactly 32 raw bytes.
fn read_seed(path: &str) -> Result<[u8; 32], String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    bytes
        .try_into()
        .map_err(|_| format!("{path}: seed must be exactly 32 bytes"))
}

/// A public key: 64 hex chars inline, or a file holding 32 raw bytes or the
/// hex form.
fn read_pubkey(arg: &str) -> Result<[u8; 32], String> {
    if let Some(key) = unhex32(arg) {
        return Ok(key);
    }
    let bytes = std::fs::read(arg).map_err(|e| format!("{arg}: {e}"))?;
    if let Ok(key) = <[u8; 32]>::try_from(bytes.as_slice()) {
        return Ok(key);
    }
    core::str::from_utf8(&bytes)
        .ok()
        .and_then(|text| unhex32(text.trim()))
        .ok_or_else(|| format!("{arg}: expected 64 hex chars or a 32-byte key file"))
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex32(text: &str) -> Option<[u8; 32]> {
    if text.len() != 64 {
        return None;
    }
    let mut out = [0u8; 32];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&text[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(out)
}
