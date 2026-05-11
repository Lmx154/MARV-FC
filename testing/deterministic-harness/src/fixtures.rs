use std::fs;
use std::path::Path;

use crate::{HarnessFailure, HarnessResult};

#[derive(Clone, Debug, PartialEq)]
pub struct FixtureSample {
    pub timestamp_us: u64,
    pub values: Vec<f32>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Fixture {
    pub name: String,
    pub columns: Vec<String>,
    pub samples: Vec<FixtureSample>,
}

impl Fixture {
    pub fn from_csv_path(path: impl AsRef<Path>) -> HarnessResult<Self> {
        let path = path.as_ref();
        let contents = fs::read_to_string(path).map_err(|error| {
            HarnessFailure::new(
                0,
                0,
                format!("failed to read fixture {}: {error}", path.display()),
            )
        })?;
        Self::from_csv_str(path.display().to_string(), &contents)
    }

    pub fn from_csv_str(name: impl Into<String>, contents: &str) -> HarnessResult<Self> {
        let name = name.into();
        let mut rows = contents
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty() && !line.starts_with('#'));

        let header = rows
            .next()
            .ok_or_else(|| HarnessFailure::new(0, 0, format!("fixture {name} is empty")))?;
        let columns = parse_header(&name, header)?;
        let mut samples = Vec::new();
        let mut previous_timestamp_us = None;

        for (line_index, line) in rows.enumerate() {
            let line_number = line_index + 2;
            let sample = parse_sample(&name, line_number, line, columns.len())?;
            if let Some(previous) = previous_timestamp_us {
                if sample.timestamp_us <= previous {
                    return Err(HarnessFailure::new(
                        samples.len() as u64,
                        sample.timestamp_us,
                        format!(
                            "fixture {name} has non-monotonic timestamp at line {line_number}: {} <= {previous}",
                            sample.timestamp_us
                        ),
                    ));
                }
            }
            previous_timestamp_us = Some(sample.timestamp_us);
            samples.push(sample);
        }

        if samples.is_empty() {
            return Err(HarnessFailure::new(
                0,
                0,
                format!("fixture {name} has no samples"),
            ));
        }

        Ok(Self {
            name,
            columns,
            samples,
        })
    }
}

fn parse_header(name: &str, header: &str) -> HarnessResult<Vec<String>> {
    let columns = split_csv_line(header);
    if columns.first().map(String::as_str) != Some("timestamp_us") {
        return Err(HarnessFailure::new(
            0,
            0,
            format!("fixture {name} first column must be timestamp_us"),
        ));
    }
    if columns.len() < 2 {
        return Err(HarnessFailure::new(
            0,
            0,
            format!("fixture {name} must include at least one value column"),
        ));
    }
    Ok(columns)
}

fn parse_sample(
    name: &str,
    line_number: usize,
    line: &str,
    expected_columns: usize,
) -> HarnessResult<FixtureSample> {
    let cells = split_csv_line(line);
    if cells.len() != expected_columns {
        return Err(HarnessFailure::new(
            line_number as u64,
            0,
            format!(
                "fixture {name} line {line_number} has {} columns, expected {expected_columns}",
                cells.len()
            ),
        ));
    }

    let timestamp_us = cells[0].parse::<u64>().map_err(|error| {
        HarnessFailure::new(
            line_number as u64,
            0,
            format!("fixture {name} line {line_number} has invalid timestamp_us: {error}"),
        )
    })?;
    let mut values = Vec::with_capacity(cells.len() - 1);
    for (column_index, cell) in cells.iter().enumerate().skip(1) {
        let value = cell.parse::<f32>().map_err(|error| {
            HarnessFailure::new(
                line_number as u64,
                timestamp_us,
                format!(
                    "fixture {name} line {line_number} column {column_index} has invalid f32: {error}"
                ),
            )
        })?;
        if !value.is_finite() {
            return Err(HarnessFailure::new(
                line_number as u64,
                timestamp_us,
                format!("fixture {name} line {line_number} column {column_index} is non-finite"),
            ));
        }
        values.push(value);
    }

    Ok(FixtureSample {
        timestamp_us,
        values,
    })
}

fn split_csv_line(line: &str) -> Vec<String> {
    line.split(',')
        .map(|cell| cell.trim().to_string())
        .collect()
}
