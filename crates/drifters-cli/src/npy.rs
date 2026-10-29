//! Reader for NumPy's `.npy` format, as far as comma2k19 needs it.
//!
//! An `.npy` file is a magic string, a version, a Python-literal header dict
//! and then the raw array. Only little-endian `f64` is accepted, which is every
//! array the replay reads. Anything else is refused by name rather than
//! reinterpreted: a byte-swapped or integer array read as `<f8` produces
//! plausible-looking garbage.
//!
//! # Both memory orders
//!
//! comma's own files are **Fortran-ordered** — column-major, so a `(n, 3)`
//! accelerometer array is stored as every x, then every y, then every z — and
//! files written by `numpy.save` today are C-ordered. The header says which.
//! [`Array::row`] always returns a row, so callers never see the difference,
//! and reading one as the other gives values that are all individually
//! reasonable and wrong together.

use std::path::Path;

use crate::kfgins::DataError;

/// A one- or two-dimensional `f64` array, held row-major.
#[derive(Clone, Debug, PartialEq)]
pub struct Array {
    /// Rows.
    pub rows: usize,
    /// Columns; 1 for a one-dimensional array.
    pub cols: usize,
    /// `rows × cols` values, row-major whatever the file's order.
    pub data: Vec<f64>,
}

impl Array {
    /// Row `i`.
    pub fn row(&self, i: usize) -> &[f64] {
        &self.data[i * self.cols..(i + 1) * self.cols]
    }

    /// Column `j`, as its own vector.
    pub fn column(&self, j: usize) -> Vec<f64> {
        (0..self.rows)
            .map(|i| self.data[i * self.cols + j])
            .collect()
    }
}

/// Read one `.npy` file.
pub fn read(path: &Path) -> Result<Array, DataError> {
    let bytes = std::fs::read(path)?;
    parse(&bytes).map_err(|reason| DataError::Parse {
        path: path.display().to_string(),
        line: 1,
        reason,
    })
}

/// Parse the bytes of an `.npy` file.
pub fn parse(bytes: &[u8]) -> Result<Array, String> {
    if bytes.len() < 10 || &bytes[..6] != b"\x93NUMPY" {
        return Err("not an .npy file".into());
    }
    // Version 1 has a two-byte header length; 2 and 3 widened it to four.
    let (start, length) = match bytes[6] {
        1 => (10, u16::from_le_bytes([bytes[8], bytes[9]]) as usize),
        2 | 3 if bytes.len() >= 12 => (
            12,
            u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]) as usize,
        ),
        v => return Err(format!("unsupported .npy version {v}")),
    };
    let header = bytes
        .get(start..start + length)
        .ok_or("header runs past the end of the file")?;
    let header = std::str::from_utf8(header).map_err(|_| "header is not text")?;

    let descr = value_of(header, "descr").ok_or("header has no descr")?;
    let descr = descr.trim_matches(|c| c == '\'' || c == '"');
    if descr != "<f8" {
        return Err(format!("dtype {descr} is not little-endian f64"));
    }
    let fortran = match value_of(header, "fortran_order").ok_or("header has no fortran_order")? {
        "True" => true,
        "False" => false,
        other => return Err(format!("fortran_order {other} is not a boolean")),
    };
    let shape = value_of(header, "shape").ok_or("header has no shape")?;
    let dims: Vec<usize> = shape
        .trim_matches(|c| c == '(' || c == ')')
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(|s| {
            s.parse()
                .map_err(|_| format!("shape {shape} is not numeric"))
        })
        .collect::<Result<_, _>>()?;
    let (rows, cols) = match dims.as_slice() {
        [] => (1, 1),
        [n] => (*n, 1),
        [n, m] => (*n, *m),
        _ => return Err(format!("shape {shape} has more than two dimensions")),
    };

    let body = &bytes[start + length..];
    let count = rows * cols;
    if body.len() != count * 8 {
        return Err(format!(
            "shape {shape} needs {} bytes of data, found {}",
            count * 8,
            body.len()
        ));
    }
    let stored: Vec<f64> = body
        .chunks_exact(8)
        .map(|c| f64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]))
        .collect();
    let data = if fortran && cols > 1 {
        let mut data = vec![0.0; count];
        for j in 0..cols {
            for i in 0..rows {
                data[i * cols + j] = stored[j * rows + i];
            }
        }
        data
    } else {
        stored
    };
    Ok(Array { rows, cols, data })
}

/// The text of `'key': value` in a header dict, up to the next top-level comma
/// or the closing brace. Tuples count as one value.
fn value_of<'a>(header: &'a str, key: &str) -> Option<&'a str> {
    let at = header.find(&format!("'{key}'"))?;
    let rest = header[at + key.len() + 2..]
        .trim_start()
        .strip_prefix(':')?;
    let rest = rest.trim_start();
    let mut depth = 0;
    for (i, c) in rest.char_indices() {
        match c {
            '(' => depth += 1,
            ')' => depth -= 1,
            ',' | '}' if depth == 0 => return Some(rest[..i].trim()),
            _ => {}
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An `.npy` file in version 1 layout, padded the way NumPy pads it.
    fn file(descr: &str, fortran: bool, shape: &str, values: &[f64]) -> Vec<u8> {
        let mut header = format!(
            "{{'descr': '{descr}', 'fortran_order': {}, 'shape': {shape}, }}",
            if fortran { "True" } else { "False" }
        );
        while (10 + header.len() + 1) % 64 != 0 {
            header.push(' ');
        }
        header.push('\n');
        let mut out = b"\x93NUMPY\x01\x00".to_vec();
        out.extend_from_slice(&(header.len() as u16).to_le_bytes());
        out.extend_from_slice(header.as_bytes());
        for v in values {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }

    #[test]
    fn a_c_ordered_matrix_reads_row_by_row() {
        let a = parse(&file("<f8", false, "(2, 3)", &[1., 2., 3., 4., 5., 6.])).unwrap();
        assert_eq!((a.rows, a.cols), (2, 3));
        assert_eq!(a.row(0), &[1., 2., 3.]);
        assert_eq!(a.row(1), &[4., 5., 6.]);
    }

    #[test]
    fn a_fortran_ordered_matrix_reads_as_the_same_rows() {
        // The same matrix, stored column by column, as comma's files are.
        let a = parse(&file("<f8", true, "(2, 3)", &[1., 4., 2., 5., 3., 6.])).unwrap();
        assert_eq!(a.row(0), &[1., 2., 3.]);
        assert_eq!(a.row(1), &[4., 5., 6.]);
        assert_eq!(a.column(1), vec![2., 5.]);
    }

    #[test]
    fn a_vector_is_one_column() {
        let a = parse(&file("<f8", false, "(3,)", &[7., 8., 9.])).unwrap();
        assert_eq!((a.rows, a.cols), (3, 1));
        assert_eq!(a.column(0), vec![7., 8., 9.]);
    }

    #[test]
    fn a_version_two_header_is_read() {
        let mut bytes = file("<f8", false, "(1,)", &[2.5]);
        // Re-encode the length as four bytes.
        let length = u16::from_le_bytes([bytes[8], bytes[9]]) as u32;
        bytes[6] = 2;
        bytes.splice(8..10, length.to_le_bytes());
        assert_eq!(parse(&bytes).unwrap().data, vec![2.5]);
    }

    #[test]
    fn anything_but_little_endian_f64_is_refused() {
        for descr in [">f8", "<f4", "<i8", "|S8"] {
            let err = parse(&file(descr, false, "(1,)", &[0.0])).unwrap_err();
            assert!(err.contains("not little-endian f64"), "{descr}: {err}");
        }
    }

    #[test]
    fn a_truncated_body_is_refused() {
        let mut bytes = file("<f8", false, "(2,)", &[1.0, 2.0]);
        bytes.truncate(bytes.len() - 1);
        assert!(parse(&bytes).unwrap_err().contains("needs 16 bytes"));
    }

    #[test]
    fn something_that_is_not_npy_is_refused() {
        assert!(parse(b"PK\x03\x04 not numpy").is_err());
        assert!(parse(b"").is_err());
    }
}
