//! `cxconvert`: raw-NMEA log -> measurement JSONL, through the real
//! `Gnss0183Driver` (the same engine `coxswain-hosted` runs live). Thin
//! over `coxswain_replay::convert`; all the actual work is there so the
//! end-to-end bridge test can call it in-process.

use std::path::PathBuf;
use std::process::ExitCode;

use coxswain_contract::SensorId;
use coxswain_drivers::gnss0183::{AcceptFilter, DEFAULT_UERE_M, SentenceKind};
use coxswain_nmea0183::Quirks;
use coxswain_replay::{ConvertConfig, convert};

const USAGE: &str = "usage: cxconvert <raw-log.jsonl> -o <measurements.jsonl> \
                     [--position-sensor <id>] [--heading-sensor <id>] \
                     [--talker <TT>]... [--sentence <GGA|RMC|HDT|VTG>]... \
                     [--checksum required|optional] [--uere-m <m>] \
                     [--fallback-std-m <m>] [--heading-std-deg <deg>]\n\n\
                     Talker/sentence flags default permissive (accept \
                     everything this driver understands); repeat to narrow.";

struct Args {
    raw_log: PathBuf,
    out: PathBuf,
    config: ConvertConfig,
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let (mut raw_log, mut out) = (None, None);
    let (mut position_sensor, mut heading_sensor) = (SensorId(1), SensorId(2));
    let mut filter = AcceptFilter::default();
    let mut checksum_required = true;
    let (mut uere_m, mut fallback_std_m, mut heading_std_deg): (f64, f64, f64) =
        (DEFAULT_UERE_M, 25.0, 0.5);

    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        let mut next = || iter.next().ok_or_else(|| USAGE.to_string());
        match arg.as_str() {
            "-o" => out = Some(PathBuf::from(next()?)),
            "--position-sensor" => {
                position_sensor = SensorId(next()?.parse().map_err(|_| USAGE.to_string())?);
            }
            "--heading-sensor" => {
                heading_sensor = SensorId(next()?.parse().map_err(|_| USAGE.to_string())?);
            }
            "--talker" => {
                let raw = next()?;
                let bytes = raw.as_bytes();
                let [a, b] = bytes else {
                    return Err(format!("--talker {raw:?}: expected exactly 2 characters"));
                };
                filter
                    .talkers
                    .push([*a, *b])
                    .map_err(|_| "--talker: too many talkers".to_string())?;
            }
            "--sentence" => {
                let kind = match next()?.as_str() {
                    "GGA" => SentenceKind::Gga,
                    "RMC" => SentenceKind::Rmc,
                    "HDT" => SentenceKind::Hdt,
                    "VTG" => SentenceKind::Vtg,
                    other => return Err(format!("--sentence {other:?}: unknown sentence type")),
                };
                filter
                    .sentences
                    .push(kind)
                    .map_err(|_| "--sentence: too many sentences".to_string())?;
            }
            "--checksum" => {
                checksum_required = match next()?.as_str() {
                    "required" => true,
                    "optional" => false,
                    other => {
                        return Err(format!("--checksum {other:?}: expected required|optional"));
                    }
                };
            }
            "--uere-m" => uere_m = next()?.parse().map_err(|_| USAGE.to_string())?,
            "--fallback-std-m" => {
                fallback_std_m = next()?.parse().map_err(|_| USAGE.to_string())?
            }
            "--heading-std-deg" => {
                heading_std_deg = next()?.parse().map_err(|_| USAGE.to_string())?;
            }
            _ if raw_log.is_none() => raw_log = Some(PathBuf::from(arg)),
            _ => return Err(USAGE.to_string()),
        }
    }
    let (Some(raw_log), Some(out)) = (raw_log, out) else {
        return Err(USAGE.to_string());
    };
    Ok(Args {
        raw_log,
        out,
        config: ConvertConfig {
            position_sensor,
            heading_sensor,
            filter,
            quirks: Quirks { checksum_required },
            uere_m,
            fallback_std_m,
            heading_std_rad: heading_std_deg.to_radians(),
        },
    })
}

fn run() -> Result<(), String> {
    let args = parse_args(&std::env::args().skip(1).collect::<Vec<_>>())?;
    let stats = convert(&args.raw_log, &args.out, &args.config)
        .map_err(|e| format!("{}: {e}", args.raw_log.display()))?;

    eprintln!(
        "cxconvert: {} lines seen, {} malformed, {} parsed, {} measurements emitted, \
         {} rejected",
        stats.lines_seen,
        stats.lines_malformed,
        stats.parsed(),
        stats.measurements_emitted,
        stats.rejected_total(),
    );
    for (reason, count) in &stats.rejected {
        eprintln!("cxconvert:   rejected as {reason}: {count}");
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("cxconvert: {message}");
            ExitCode::FAILURE
        }
    }
}
