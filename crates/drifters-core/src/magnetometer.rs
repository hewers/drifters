//! Three-axis magnetometer calibration.
//!
//! A magnetometer bolted into anything reads the Earth's field distorted by
//! what is around it. Permanent magnetism adds a constant, the *hard iron*.
//! Soft magnetic material bends and scales the field, the *soft iron*, and the
//! sensor's own gain and axis errors have the same form. Together:
//!
//! ```text
//! raw = A·m_b + b + noise        m_b = S·(raw − b),   S = A⁻¹
//! ```
//!
//! [`MagCalibration`] is `(b, S)`. Two fits find it, and they answer different
//! situations:
//!
//! - [`EllipsoidFit`] needs nothing but the readings. As the sensor turns, `raw`
//!   traces an ellipsoid, and the fit finds the map that makes it a sphere. It
//!   needs the sensor turned through a real spread of directions, which a hand
//!   or an aircraft can do and a car cannot. An ellipsoid has no opinion about
//!   rotation, so `S` is symmetric, and any rotation between the magnetometer
//!   and the IMU survives it.
//! - [`AidedMagFit`] uses an attitude from a filter that already has one, and
//!   a reference field. Then `raw` is linear in `(A, b)`, the solve is `3 × 3`,
//!   and the rotation between the sensor and the body comes out too. It works
//!   on a vehicle that only ever turns flat. That motion shows it two axes, and
//!   the third is completed as the rotation the other two imply.
//!
//! Both refuse rather than guess: too few samples, too little of the sphere
//! seen, a residual that says the model does not fit, or a distortion too large
//! to believe give `None`.
//!
//! # What a fit cannot do
//!
//! A calibration is valid for the magnetic surroundings it was made in. A
//! current that switches, a door that opens or a change of payload moves it,
//! and neither fit can see that afterwards. [`MagFit::is_plausible`] catches the
//! cases that change the field's strength. The filter's gate catches those that
//! move its direction quickly and by more than it allows. One that comes on
//! slowly and keeps the strength is followed, and costs more than having no
//! magnetometer would.
//!
//! The same goes for a sensor turned in its mount, which is why the rotation
//! matters. In the simulated flights of `docs/testing.md`, three degrees that
//! an [`EllipsoidFit`] left in place ended a five-minute GNSS outage 36° off in
//! heading, against 4° with no magnetometer and 1.9° with an [`AidedMagFit`].

// A refusal here has to happen for NaN as well, and `!(a <= b)` says so where
// `a > b` would let it through.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use crate::estimator::PvaStd;
#[cfg_attr(any(test, feature = "std", feature = "serde"), allow(unused_imports))]
use crate::math::Real;
use crate::math::{Cholesky, Mat3, Matrix, Vec3};
use crate::types::{has_direction, NavState};
use crate::F;

/// Sweeps of the Jacobi eigenvalue iteration. A symmetric `3 × 3` converges to
/// machine precision in five or six.
const JACOBI_SWEEPS: usize = 12;

/// The Earth's field in the local NED frame, from the three numbers a magnetic
/// model or chart gives.
///
/// `declination` is east of true north and `inclination` is down from the
/// horizontal, both radians. `strength` sets the units of the result.
pub fn field_from_angles(declination: F, inclination: F, strength: F) -> Vec3 {
    let (sin_i, cos_i) = inclination.sin_cos();
    let (sin_d, cos_d) = declination.sin_cos();
    Vec3::new(cos_i * cos_d, cos_i * sin_d, sin_i) * strength
}

/// The correction `m = S·(raw − b)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagCalibration {
    /// The offset `b`, in the sensor's raw units.
    pub hard_iron: Vec3,
    /// The matrix `S`. It may include a rotation, which then also carries the
    /// reading from the sensor's axes into the body frame.
    pub soft_iron: Mat3,
}

impl MagCalibration {
    /// No correction at all.
    pub const IDENTITY: Self = Self {
        hard_iron: Vec3::new(0.0, 0.0, 0.0),
        soft_iron: Matrix::from_rows([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
    };

    /// True when every element is finite and `S` can be inverted, so that
    /// distinct fields stay distinct.
    pub fn is_valid(&self) -> bool {
        let det = determinant(&self.soft_iron);
        self.hard_iron.is_finite() && self.soft_iron.is_finite() && det != 0.0 && det.is_finite()
    }

    /// The corrected field for a raw reading.
    #[inline]
    pub fn apply(&self, raw: Vec3) -> Vec3 {
        self.soft_iron * (raw - self.hard_iron)
    }
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// When a fit is willing to answer.
///
/// `#[non_exhaustive]` (`docs/adr/0010`): start from [`Default`] and assign the
/// thresholds to change. A threshold that is not a number admits nothing.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
pub struct MagFitConfig {
    /// Samples needed before there is an answer.
    pub min_samples: u32,
    /// Least spread of the field directions seen, as a standard deviation on
    /// the unit sphere. Evenly over the sphere is 0.58 on every axis, half of
    /// it is 0.29 on the worst one, and a flat turn is zero on one axis.
    ///
    /// [`EllipsoidFit`] needs this on all three axes. [`AidedMagFit`] needs it
    /// on two, and fills in a third that falls short.
    pub min_spread: F,
    /// Largest RMS misfit, as a fraction of the field's strength.
    pub max_residual: F,
    /// Largest ratio between the distortion's strongest and weakest axes.
    pub max_axis_ratio: F,
    /// Largest attitude sigma [`AidedMagFit`] will take a sample at, radians.
    pub max_attitude_std: F,
}

impl Default for MagFitConfig {
    fn default() -> Self {
        Self {
            min_samples: 100,
            min_spread: 0.2,
            max_residual: 0.05,
            max_axis_ratio: 3.0,
            max_attitude_std: 1.0_f64.to_radians() as F,
        }
    }
}

/// What a fit concluded.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagFit {
    /// The correction.
    pub calibration: MagCalibration,
    /// The field strength a corrected reading should have. [`EllipsoidFit`]
    /// leaves it in the sensor's raw units, and [`AidedMagFit`] in the units of
    /// the reference it was given.
    pub strength: F,
    /// RMS misfit, as a fraction of `strength`.
    pub residual: F,
    /// Samples behind it.
    pub samples: u32,
    /// How many axes the motion actually showed: 3, or 2 when [`AidedMagFit`]
    /// completed the last one.
    pub observed_axes: u8,
}

impl MagFit {
    /// True when `calibrated` has about the strength this fit expects, within
    /// `tolerance` as a fraction.
    ///
    /// A nearby magnet or current usually changes the strength, and it is the
    /// check that needs no attitude. It does not catch a distortion that only
    /// turns the field.
    pub fn is_plausible(&self, calibrated: Vec3, tolerance: F) -> bool {
        (calibrated.norm() / self.strength - 1.0).abs() <= tolerance
    }
}

/// Fits the ellipsoid the raw readings lie on, with no other information.
///
/// Allocation-free, and it keeps sums rather than samples, so it can run for as
/// long as the sensor is being turned. A sample far off the ellipsoid spoils
/// the sums and shows up as a residual that refuses the fit, after which
/// [`reset`](Self::reset) starts again.
///
/// The surface is a general quadric with its trace fixed, which makes the least
/// squares problem linear and independent of how the sensor's axes point:
///
/// ```text
/// x²+y²+z² = u₀(x²+y²−2z²) + u₁(x²+z²−2y²) + 2u₂xy + 2u₃xz + 2u₄yz
///          + 2u₅x + 2u₆y + 2u₇z + u₈
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EllipsoidFit {
    config: MagFitConfig,
    /// The first sample. Everything is summed relative to it, so a large hard
    /// iron costs no precision.
    origin: Option<Vec3>,
    normal: Matrix<9, 9>,
    right: Matrix<9, 1>,
    /// `Σ d²`, for the residual.
    target: F,
    first: Vec3,
    second: Mat3,
    samples: u32,
}

impl Default for EllipsoidFit {
    fn default() -> Self {
        Self::new(MagFitConfig::default())
    }
}

impl EllipsoidFit {
    /// A fit with nothing seen yet.
    pub const fn new(config: MagFitConfig) -> Self {
        Self {
            config,
            origin: None,
            normal: Matrix::zeros(),
            right: Matrix::zeros(),
            target: 0.0,
            first: Vec3::new(0.0, 0.0, 0.0),
            second: Matrix::zeros(),
            samples: 0,
        }
    }

    /// Forget everything seen, keeping the configuration.
    pub fn reset(&mut self) {
        *self = Self::new(self.config);
    }

    /// Samples taken so far.
    #[inline]
    pub fn samples(&self) -> u32 {
        self.samples
    }

    /// Take a raw reading. Returns whether it was used: one that is not finite,
    /// or so large that its powers are not, is not.
    pub fn add(&mut self, raw: Vec3) -> bool {
        if self.samples == u32::MAX || !usable_reading(raw) {
            return false;
        }
        let origin = *self.origin.get_or_insert(raw);
        let p = raw - origin;
        if !usable_reading(p) {
            return false;
        }
        let (x, y, z) = (p.x, p.y, p.z);
        let (xx, yy, zz) = (x * x, y * y, z * z);
        let d = xx + yy + zz;
        let row = [
            xx + yy - 2.0 * zz,
            xx + zz - 2.0 * yy,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            2.0 * x,
            2.0 * y,
            2.0 * z,
            1.0,
        ];
        let rows = self.normal.data.iter_mut().zip(self.right.data.iter_mut());
        for ((normal, right), a) in rows.zip(row) {
            for (cell, b) in normal.iter_mut().zip(row) {
                *cell += a * b;
            }
            for cell in right.iter_mut() {
                *cell += a * d;
            }
        }
        self.target += d * d;
        self.first += p;
        self.second += outer(p, p);
        self.samples += 1;
        true
    }

    /// The calibration, if what has been seen supports one.
    pub fn solve(&self) -> Option<MagFit> {
        if self.samples < self.config.min_samples {
            return None;
        }
        let origin = self.origin?;
        let n = self.samples as F;
        let u = solve_equilibrated(&self.normal, &self.right)?;
        let [u0, u1, d, e, f, g, h, i, j] = u;

        // Back to `xᵀQx + 2qᵀx + j = 0`, whose Q has trace −3 by construction.
        let q = Mat3::from_rows([
            [u0 + u1 - 1.0, d, e],
            [d, u0 - 2.0 * u1 - 1.0, f],
            [e, f, u1 - 2.0 * u0 - 1.0],
        ]);
        let linear = Vec3::new(g, h, i);
        // −Q must be positive definite for the surface to be an ellipsoid at
        // all. Its factor also gives the centre, from Q·c = −q.
        let negated = q.scaled(-1.0);
        let centre = from_column(&Cholesky::new(&negated)?.solve(&to_column(linear)));
        // `(x−c)ᵀQ(x−c) = k`, and k is negative for a real ellipsoid.
        let k = centre.dot(q * centre) - j;
        if !(k < 0.0) {
            return None;
        }

        let shape = q.scaled(1.0 / k);
        let (values, vectors) = symmetric_eigen(&shape);
        let [l0, l1, l2] = values;
        if !(l0 > 0.0 && l1 > 0.0 && l2 > 0.0) {
            return None;
        }
        // Semi-axes are 1/√λ, and λ is sorted ascending.
        if !ratio_is_believable(l2, l0, self.config.max_axis_ratio) {
            return None;
        }
        // The sphere with the ellipsoid's volume, so the result keeps the
        // sensor's units and a determinant of one.
        let radius = (l0 * l1 * l2).powf(-1.0 / 6.0);
        let root = Vec3::new(l0.sqrt(), l1.sqrt(), l2.sqrt()) * radius;
        let soft_iron = vectors.matmul(&root.to_diag()).mul_transpose(&vectors);

        // The algebraic error is `k·(|S(x−c)|²/r² − 1)`, so half of it over k
        // is the fractional error in length.
        let explained: F = u
            .iter()
            .zip(self.right.data.iter())
            .map(|(a, b)| a * b[0])
            .sum();
        let squared = (self.target - explained) / n;
        let residual = positive_sqrt(squared) / (2.0 * k.abs());
        if !(residual <= self.config.max_residual) {
            return None;
        }

        // How much of the sphere was seen, measured on the sphere.
        let mean = self.first * (1.0 / n);
        let spread = self.second.scaled(1.0 / n) - outer(mean, mean);
        let on_sphere = soft_iron
            .matmul(&spread)
            .mul_transpose(&soft_iron)
            .scaled(1.0 / (radius * radius));
        let (seen, _) = symmetric_eigen(&on_sphere);
        let least = self.config.min_spread;
        if !(seen[0] >= least * least) {
            return None;
        }

        let calibration = MagCalibration {
            hard_iron: origin + centre,
            soft_iron,
        };
        (calibration.is_valid() && radius.is_finite()).then_some(MagFit {
            calibration,
            strength: radius,
            residual,
            samples: self.samples,
            observed_axes: 3,
        })
    }
}

/// Fits the distortion against an attitude the filter already trusts.
///
/// Feed it from a filter being aided by something other than this magnetometer,
/// or the attitude it is given has already been pulled toward the error being
/// measured. With `raw = A·h + b` and `h` the reference field in the body frame,
/// the problem is three small linear regressions sharing one matrix.
///
/// Whatever `h` did not vary along cannot be told from the hard iron. A vehicle
/// turning flat sees it fixed along the vertical axis. There the gain is taken
/// to match the axes that were seen and the direction to complete a right-handed
/// frame from them. [`MagFit::observed_axes`] says that happened, and the result
/// is then good while the vehicle stays near that attitude.
///
/// An error in the reference's declination is a rotation about the vertical.
/// For a vehicle that stays level it commutes with every heading, so the fit
/// takes it for a turn of the sensor about the body's down axis, and the
/// corrected readings agree with the reference as given. That is the useful
/// answer, since the filter compares them with the same reference. A vehicle
/// that tumbles cannot be explained that way, and the error appears in
/// [`MagFit::residual`], at a little over half the angle.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AidedMagFit {
    config: MagFitConfig,
    sum_field: Vec3,
    sum_raw: Vec3,
    field_field: Mat3,
    raw_field: Mat3,
    raw_raw: F,
    samples: u32,
}

impl Default for AidedMagFit {
    fn default() -> Self {
        Self::new(MagFitConfig::default())
    }
}

impl AidedMagFit {
    /// A fit with nothing seen yet.
    pub const fn new(config: MagFitConfig) -> Self {
        Self {
            config,
            sum_field: Vec3::new(0.0, 0.0, 0.0),
            sum_raw: Vec3::new(0.0, 0.0, 0.0),
            field_field: Matrix::zeros(),
            raw_field: Matrix::zeros(),
            raw_raw: 0.0,
            samples: 0,
        }
    }

    /// Forget everything seen, keeping the configuration.
    pub fn reset(&mut self) {
        *self = Self::new(self.config);
    }

    /// Samples taken so far.
    #[inline]
    pub fn samples(&self) -> u32 {
        self.samples
    }

    /// Take a raw reading with the state it was made in and the reference
    /// field in the NED frame. Returns whether it was used: the attitude must
    /// be known to within [`MagFitConfig::max_attitude_std`] on every axis.
    pub fn update(&mut self, state: &NavState, std: &PvaStd, raw: Vec3, reference: Vec3) -> bool {
        let limit = self.config.max_attitude_std;
        let known = std.attitude.x <= limit && std.attitude.y <= limit && std.attitude.z <= limit;
        let usable = known
            && self.samples < u32::MAX
            && state.is_finite()
            && usable_reading(raw)
            && has_direction(reference);
        if !usable {
            return false;
        }
        let h = state.pva.attitude.quat().rotate_inverse(reference);
        self.sum_field += h;
        self.sum_raw += raw;
        self.field_field += outer(h, h);
        self.raw_field += outer(raw, h);
        self.raw_raw += raw.norm_squared();
        self.samples += 1;
        true
    }

    /// The calibration, if what has been seen supports one.
    pub fn solve(&self) -> Option<MagFit> {
        if self.samples < self.config.min_samples {
            return None;
        }
        let scale = 1.0 / self.samples as F;
        let mean_field = self.sum_field * scale;
        let mean_raw = self.sum_raw * scale;
        let field = self.field_field.scaled(scale) - outer(mean_field, mean_field);
        let cross = self.raw_field.scaled(scale) - outer(mean_raw, mean_field);
        let power = self.field_field.trace() * scale;
        if !(field.is_finite() && cross.is_finite() && power > 0.0 && power.is_finite()) {
            return None;
        }

        let (values, vectors) = symmetric_eigen(&field);
        let least = self.config.min_spread * self.config.min_spread * power;
        let v = [vectors.column(0), vectors.column(1), vectors.column(2)];
        // λ is ascending, so the axes seen are the last ones.
        let image = |k: usize| cross * v[k] * (1.0 / values[k]);
        if !(values[1] >= least) {
            return None;
        }
        let (seen, a0) = if values[0] >= least {
            (3, image(0))
        } else {
            // Scaled like the two seen, and turned as they are turned. The
            // eigenvectors are right-handed, so v₀ = v₁ × v₂ and its image is
            // the same cross product of theirs.
            let (a1, a2) = (image(1), image(2));
            let gain = ((a1.norm_squared() + a2.norm_squared()) * 0.5).sqrt();
            (2, a1.cross(a2).normalized() * gain)
        };
        let a = from_images(&v, [a0, image(1), image(2)]);

        let (stretch, _) = symmetric_eigen(&a.transpose().matmul(&a));
        if !(stretch[0] > 0.0)
            || !ratio_is_believable(stretch[2], stretch[0], self.config.max_axis_ratio)
        {
            return None;
        }
        let soft_iron = inverse(&a)?;
        let hard_iron = mean_raw - a * mean_field;

        // `E|r̃ − A·h̃|²`, expanded so that it needs only the sums.
        let spread = self.raw_raw * scale - mean_raw.norm_squared();
        let fitted = a.transpose().matmul(&cross).trace();
        let modelled = a.matmul(&field).mul_transpose(&a).trace();
        let misfit = positive_sqrt(spread - 2.0 * fitted + modelled);
        let strength = power.sqrt();
        // The misfit is in raw units, so it is compared with the field in them.
        let raw_strength = ((stretch[0] + stretch[1] + stretch[2]) / 3.0).sqrt() * strength;
        let residual = misfit / raw_strength;
        if !(residual <= self.config.max_residual) {
            return None;
        }

        let calibration = MagCalibration {
            hard_iron,
            soft_iron,
        };
        calibration.is_valid().then_some(MagFit {
            calibration,
            strength,
            residual,
            samples: self.samples,
            observed_axes: seen,
        })
    }
}

/// True when a reading and the fourth power of its size are finite.
fn usable_reading(v: Vec3) -> bool {
    let squared = v.norm_squared();
    v.is_finite() && (squared * squared).is_finite()
}

/// `√x`, reading anything not above zero (rounding, or NaN) as zero.
fn positive_sqrt(x: F) -> F {
    if x > 0.0 {
        x.sqrt()
    } else {
        0.0
    }
}

/// True when the axes behind eigenvalues `largest` and `smallest` of a squared
/// quantity differ by no more than `limit`.
fn ratio_is_believable(largest: F, smallest: F, limit: F) -> bool {
    largest <= limit * limit * smallest
}

/// Solve the symmetric positive definite `N·x = r` after scaling `N` to a unit
/// diagonal. The nine columns of the ellipsoid fit differ in size by powers of
/// the field, and this takes that out of the conditioning.
fn solve_equilibrated(normal: &Matrix<9, 9>, right: &Matrix<9, 1>) -> Option<[F; 9]> {
    let mut scale = [0.0; 9];
    for (i, (s, row)) in scale.iter_mut().zip(normal.data.iter()).enumerate() {
        let d = row.get(i).copied()?;
        if !(d > 0.0 && d.is_finite()) {
            return None;
        }
        *s = 1.0 / d.sqrt();
    }
    let mut scaled = *normal;
    let mut rhs = *right;
    for ((row, r), si) in scaled.data.iter_mut().zip(rhs.data.iter_mut()).zip(scale) {
        for (cell, sj) in row.iter_mut().zip(scale) {
            *cell *= si * sj;
        }
        r[0] *= si;
    }
    let solution = Cholesky::new(&scaled)?.solve(&rhs);
    let mut x = [0.0; 9];
    for ((out, s), v) in x.iter_mut().zip(scale).zip(solution.data.iter()) {
        *out = v[0] * s;
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

#[inline]
fn outer(a: Vec3, b: Vec3) -> Mat3 {
    Mat3::from_rows([
        [a.x * b.x, a.x * b.y, a.x * b.z],
        [a.y * b.x, a.y * b.y, a.y * b.z],
        [a.z * b.x, a.z * b.y, a.z * b.z],
    ])
}

#[inline]
fn to_column(v: Vec3) -> Matrix<3, 1> {
    Matrix::from_column([v.x, v.y, v.z])
}

#[inline]
fn from_column(m: &Matrix<3, 1>) -> Vec3 {
    let [x, y, z] = m.to_column();
    Vec3::new(x, y, z)
}

/// The matrix sending each of the orthonormal `axes` to its `image`.
fn from_images(axes: &[Vec3; 3], images: [Vec3; 3]) -> Mat3 {
    outer(images[0], axes[0]) + outer(images[1], axes[1]) + outer(images[2], axes[2])
}

fn determinant(m: &Mat3) -> F {
    m.column(0).dot(m.column(1).cross(m.column(2)))
}

/// The inverse, by cofactors, or `None` for a singular matrix.
fn inverse(m: &Mat3) -> Option<Mat3> {
    let (a, b, c) = (m.column(0), m.column(1), m.column(2));
    let det = a.dot(b.cross(c));
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    let (x, y, z) = (b.cross(c), c.cross(a), a.cross(b));
    Some(Mat3::from_rows([[x.x, x.y, x.z], [y.x, y.y, y.z], [z.x, z.y, z.z]]).scaled(1.0 / det))
}

/// One Jacobi rotation zeroing element `(P, Q)`. The indices are constants so
/// that every access below is provably in range.
#[inline]
fn jacobi_rotate<const P: usize, const Q: usize>(a: &mut Mat3, v: &mut Mat3) {
    let apq = a[(P, Q)];
    if apq == 0.0 {
        return;
    }
    let theta = (a[(Q, Q)] - a[(P, P)]) / (2.0 * apq);
    let t = if theta >= 0.0 { 1.0 } else { -1.0 } / (theta.abs() + (theta * theta + 1.0).sqrt());
    let c = 1.0 / (t * t + 1.0).sqrt();
    let s = t * c;
    for k in 0..3 {
        let (akp, akq) = (a[(k, P)], a[(k, Q)]);
        a[(k, P)] = c * akp - s * akq;
        a[(k, Q)] = s * akp + c * akq;
    }
    for k in 0..3 {
        let (apk, aqk) = (a[(P, k)], a[(Q, k)]);
        a[(P, k)] = c * apk - s * aqk;
        a[(Q, k)] = s * apk + c * aqk;
    }
    for k in 0..3 {
        let (vkp, vkq) = (v[(k, P)], v[(k, Q)]);
        v[(k, P)] = c * vkp - s * vkq;
        v[(k, Q)] = s * vkp + c * vkq;
    }
}

/// Eigenvalues in ascending order, and the matching eigenvectors as the columns
/// of a proper rotation.
fn symmetric_eigen(m: &Mat3) -> ([F; 3], Mat3) {
    let mut a = *m;
    let mut v = Mat3::identity();
    for _ in 0..JACOBI_SWEEPS {
        let off = a[(0, 1)] * a[(0, 1)] + a[(0, 2)] * a[(0, 2)] + a[(1, 2)] * a[(1, 2)];
        // `!(>)` so that NaN stops the iteration too.
        if !(off > 0.0) {
            break;
        }
        jacobi_rotate::<0, 1>(&mut a, &mut v);
        jacobi_rotate::<0, 2>(&mut a, &mut v);
        jacobi_rotate::<1, 2>(&mut a, &mut v);
    }
    let mut pairs = [
        (a[(0, 0)], v.column(0)),
        (a[(1, 1)], v.column(1)),
        (a[(2, 2)], v.column(2)),
    ];
    // Three elements: an exchange sort needs no comparator that could panic.
    if pairs[1].0 < pairs[0].0 {
        pairs.swap(0, 1);
    }
    if pairs[2].0 < pairs[1].0 {
        pairs.swap(1, 2);
    }
    if pairs[1].0 < pairs[0].0 {
        pairs.swap(0, 1);
    }
    let [(l0, v0), (l1, v1), (l2, v2)] = pairs;
    // Sorting may have reflected the frame. The third axis restores it.
    let v2 = if v0.cross(v1).dot(v2) < 0.0 {
        v2 * -1.0
    } else {
        v2
    };
    ([l0, l1, l2], Mat3::from_columns(v0, v1, v2))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Quat;
    use crate::time::GpsTime;
    use crate::types::{Attitude, ImuError, Pva};
    use approx::assert_relative_eq;

    /// A small deterministic generator, so that a failure can be reproduced.
    struct Rng(u64);

    impl Rng {
        fn uniform(&mut self) -> F {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 33) as F) / ((1_u64 << 31) as F)
        }

        fn normal(&mut self) -> F {
            let (a, b) = (self.uniform().max(1e-12), self.uniform());
            (-2.0 * a.ln()).sqrt() * (core::f64::consts::TAU * b).cos()
        }

        fn vector(&mut self, sigma: F) -> Vec3 {
            Vec3::new(self.normal(), self.normal(), self.normal()) * sigma
        }

        fn direction(&mut self) -> Vec3 {
            self.vector(1.0).normalized()
        }
    }

    fn truth() -> (Mat3, Vec3) {
        // Gains of 0.8 to 1.25 with cross-coupling, and an offset larger than
        // the field, which is what a magnet near the sensor looks like.
        let a = Mat3::from_rows([[1.25, 0.08, -0.05], [0.08, 0.85, 0.11], [-0.05, 0.11, 1.05]]);
        (a, Vec3::new(70.0, -35.0, 120.0))
    }

    const STRENGTH: F = 50.0;

    /// Angle between `field` and what the calibration makes of its reading.
    fn direction_error(fit: &MagFit, a: &Mat3, b: Vec3, field: Vec3) -> F {
        let corrected = fit.calibration.apply(*a * field + b);
        corrected
            .normalized()
            .cross(field.normalized())
            .norm()
            .asin()
    }

    fn sphere_fit(seed: u64, noise: F) -> EllipsoidFit {
        let (a, b) = truth();
        let mut rng = Rng(seed);
        let mut fit = EllipsoidFit::default();
        for _ in 0..600 {
            let field = rng.direction() * STRENGTH;
            let raw = a * field + b + rng.vector(noise);
            assert!(fit.add(raw));
        }
        fit
    }

    #[test]
    fn a_clean_sphere_is_recovered_exactly() {
        let (a, b) = truth();
        let fit = sphere_fit(1, 0.0).solve().expect("a full sphere");
        assert_eq!(fit.observed_axes, 3);
        assert!(fit.residual < 1e-7, "residual {}", fit.residual);
        assert_relative_eq!(fit.calibration.hard_iron.x, b.x, epsilon = 1e-6);
        assert_relative_eq!(fit.calibration.hard_iron.y, b.y, epsilon = 1e-6);
        assert_relative_eq!(fit.calibration.hard_iron.z, b.z, epsilon = 1e-6);
        // S·A is a rotation scaled by strength over the raw radius: S is only
        // defined up to one, and it must be one.
        let product = fit.calibration.soft_iron.matmul(&a);
        let scale = fit.strength / STRENGTH;
        let shouldbe_identity = product
            .mul_transpose(&product)
            .scaled(1.0 / (scale * scale));
        assert!((shouldbe_identity - Mat3::identity()).amax() < 1e-9);
        assert_relative_eq!(determinant(&fit.calibration.soft_iron), 1.0, epsilon = 1e-9);
    }

    #[test]
    fn a_noisy_sphere_gives_a_field_of_constant_length() {
        let (a, b) = truth();
        let noise = 0.5;
        let fit = sphere_fit(2, noise).solve().expect("a full sphere");
        let mut rng = Rng(99);
        let mut worst: F = 0.0;
        for _ in 0..500 {
            let raw = a * (rng.direction() * STRENGTH) + b;
            let length = fit.calibration.apply(raw).norm();
            worst = worst.max((length / fit.strength - 1.0).abs());
        }
        assert!(worst < 0.01, "length varies by {worst}");
        assert!(fit.residual < 0.02, "residual {}", fit.residual);
    }

    #[test]
    fn the_answer_does_not_depend_on_the_order_of_the_samples_or_their_units() {
        let (a, b) = truth();
        let mut rng = Rng(3);
        let readings: [Vec3; 300] =
            core::array::from_fn(|_| a * (rng.direction() * STRENGTH) + b + rng.vector(0.2));
        let run = |scale: F, reversed: bool| {
            let mut fit = EllipsoidFit::default();
            for k in 0..readings.len() {
                let i = if reversed { readings.len() - 1 - k } else { k };
                assert!(fit.add(readings[i] * scale));
            }
            fit.solve().expect("a full sphere")
        };
        let plain = run(1.0, false);
        let reversed = run(1.0, true);
        let in_gauss = run(0.01, false);
        assert!((plain.calibration.soft_iron - reversed.calibration.soft_iron).amax() < 1e-9);
        assert!((plain.calibration.soft_iron - in_gauss.calibration.soft_iron).amax() < 1e-9);
        assert_relative_eq!(
            in_gauss.strength,
            plain.strength * 0.01,
            max_relative = 1e-9
        );
    }

    #[test]
    fn a_flat_turn_is_not_enough_for_an_ellipsoid() {
        let (a, b) = truth();
        let mut rng = Rng(4);
        let mut fit = EllipsoidFit::default();
        let dip: F = 1.0;
        for k in 0..1000 {
            let yaw = k as F * 0.01;
            let field = Vec3::new(dip.cos() * yaw.cos(), dip.cos() * yaw.sin(), dip.sin());
            assert!(fit.add(a * (field * STRENGTH) + b + rng.vector(0.2)));
        }
        assert_eq!(fit.solve(), None);
    }

    #[test]
    fn a_small_patch_of_the_sphere_is_not_enough() {
        let (a, b) = truth();
        let mut rng = Rng(5);
        let mut fit = EllipsoidFit::default();
        for _ in 0..1000 {
            let wobble = rng.vector(0.05);
            let field = (Vec3::new(1.0, 0.0, 0.0) + wobble).normalized();
            assert!(fit.add(a * (field * STRENGTH) + b + rng.vector(0.2)));
        }
        assert_eq!(fit.solve(), None);
    }

    #[test]
    fn too_few_samples_give_nothing_and_reset_starts_again() {
        let (a, b) = truth();
        let mut rng = Rng(6);
        let mut fit = EllipsoidFit::default();
        for _ in 0..99 {
            fit.add(a * (rng.direction() * STRENGTH) + b);
        }
        assert_eq!(fit.solve(), None);
        fit.add(a * (rng.direction() * STRENGTH) + b);
        assert!(fit.solve().is_some());
        fit.reset();
        assert_eq!(fit.samples(), 0);
        assert_eq!(fit.solve(), None);
    }

    #[test]
    fn readings_that_are_not_numbers_are_refused() {
        let mut fit = EllipsoidFit::default();
        assert!(!fit.add(Vec3::new(F::NAN, 0.0, 0.0)));
        assert!(!fit.add(Vec3::splat(F::INFINITY)));
        // Finite, but its fourth power is not.
        assert!(!fit.add(Vec3::splat(1e100)));
        assert_eq!(fit.samples(), 0);
    }

    #[test]
    fn a_stray_reading_is_refused_by_the_residual() {
        let (a, b) = truth();
        let mut rng = Rng(7);
        let mut fit = EllipsoidFit::default();
        for k in 0..500 {
            let field = rng.direction() * STRENGTH;
            let stray = if k % 5 == 0 { 1.6 } else { 1.0 };
            fit.add(a * (field * stray) + b);
        }
        assert_eq!(fit.solve(), None);
    }

    #[test]
    fn a_distortion_too_large_to_believe_is_refused() {
        let mut rng = Rng(8);
        let a = Vec3::new(4.0, 1.0, 1.0).to_diag();
        let mut fit = EllipsoidFit::default();
        for _ in 0..500 {
            fit.add(a * (rng.direction() * STRENGTH));
        }
        assert_eq!(fit.solve(), None);
    }

    #[test]
    fn half_a_sphere_is_enough() {
        let (a, b) = truth();
        let mut rng = Rng(9);
        let mut fit = EllipsoidFit::default();
        for _ in 0..800 {
            let mut field = rng.direction();
            field.z = field.z.abs();
            fit.add(a * (field * STRENGTH) + b + rng.vector(0.1));
        }
        let found = fit.solve().expect("half a sphere");
        let mut worst: F = 0.0;
        for _ in 0..200 {
            let raw = a * (rng.direction() * STRENGTH) + b;
            let length = found.calibration.apply(raw).norm();
            worst = worst.max((length / found.strength - 1.0).abs());
        }
        assert!(worst < 0.02, "length varies by {worst}");
    }

    fn state_at(attitude: Quat) -> NavState {
        NavState {
            time: GpsTime::from_tow(10.0),
            pva: Pva {
                attitude: Attitude::from_quat(attitude),
                ..Pva::default()
            },
            imu_error: ImuError::ZERO,
        }
    }

    fn known() -> PvaStd {
        PvaStd {
            position: Vec3::splat(1.0),
            velocity: Vec3::splat(0.1),
            attitude: Vec3::splat(0.001),
        }
    }

    fn reference() -> Vec3 {
        field_from_angles(0.05, 1.0, STRENGTH)
    }

    /// The sensor's axes turned against the body by this.
    fn mount() -> Mat3 {
        Quat::from_euler(0.2, -0.1, 0.4).to_dcm()
    }

    #[test]
    fn the_aided_fit_recovers_a_rotated_sensor_from_a_tumble() {
        let (a, b) = truth();
        let a = a.matmul(&mount());
        let mut rng = Rng(10);
        let mut fit = AidedMagFit::default();
        for _ in 0..400 {
            let attitude = Quat::from_rotation_vector(rng.vector(1.5));
            let h = attitude.rotate_inverse(reference());
            assert!(fit.update(&state_at(attitude), &known(), a * h + b, reference()));
        }
        let found = fit.solve().expect("every axis seen");
        assert_eq!(found.observed_axes, 3);
        assert!(found.residual < 1e-6, "residual {}", found.residual);
        assert_relative_eq!(found.strength, STRENGTH, max_relative = 1e-9);
        // Unlike the ellipsoid, this one knows S exactly, rotation included.
        let product = found.calibration.soft_iron.matmul(&a);
        assert!((product - Mat3::identity()).amax() < 1e-9);
        assert!(direction_error(&found, &a, b, Vec3::new(1.0, 2.0, 3.0)) < 1e-9);
    }

    /// Readings from a vehicle turning flat, with `tilt` of bank and pitch.
    fn flat_turns(a: &Mat3, b: Vec3, tilt: F, noise: F) -> AidedMagFit {
        let mut rng = Rng(11);
        let mut fit = AidedMagFit::default();
        for k in 0..2000 {
            let yaw = k as F * 0.0123;
            let roll = tilt * (k as F * 0.07).sin();
            let pitch = tilt * (k as F * 0.05).cos();
            let attitude = Quat::from_euler(roll, pitch, yaw);
            let h = attitude.rotate_inverse(reference());
            let raw = *a * h + b + rng.vector(noise);
            assert!(fit.update(&state_at(attitude), &known(), raw, reference()));
        }
        fit
    }

    #[test]
    fn a_flat_turn_gives_the_aided_fit_two_axes_and_a_usable_answer() {
        let (a, b) = truth();
        let a = a.matmul(&Quat::from_euler(0.0, 0.0, 0.3).to_dcm());
        let found = flat_turns(&a, b, 0.0, 0.1).solve().expect("two axes seen");
        assert_eq!(found.observed_axes, 2);
        // Over the attitudes it was fitted at, the answer is right.
        let mut worst: F = 0.0;
        for k in 0..100 {
            let attitude = Quat::from_euler(0.0, 0.0, k as F * 0.0628);
            let h = attitude.rotate_inverse(reference());
            worst = worst.max(direction_error(&found, &a, b, h));
        }
        assert!(worst < 0.01, "worst direction error {worst} rad");
    }

    #[test]
    fn the_aided_fit_needs_more_than_one_axis() {
        let (a, b) = truth();
        let mut fit = AidedMagFit::default();
        for k in 0..500 {
            // Turning about the field itself: nothing changes at all.
            let attitude = Quat::from_rotation_vector(reference().normalized() * (k as F * 0.01));
            let h = attitude.rotate_inverse(reference());
            assert!(fit.update(&state_at(attitude), &known(), a * h + b, reference()));
        }
        assert_eq!(fit.solve(), None);
    }

    #[test]
    fn the_aided_fit_ignores_a_filter_unsure_of_its_attitude() {
        let (a, b) = truth();
        let mut fit = AidedMagFit::default();
        let mut std = known();
        std.attitude.z = 0.1;
        let state = state_at(Quat::IDENTITY);
        let h = reference();
        assert!(!fit.update(&state, &std, a * h + b, reference()));
        std.attitude.z = F::NAN;
        assert!(!fit.update(&state, &std, a * h + b, reference()));
        assert!(!fit.update(&state, &known(), Vec3::splat(F::NAN), reference()));
        assert!(!fit.update(&state, &known(), a * h + b, Vec3::ZERO));
        assert_eq!(fit.samples(), 0);
    }

    #[test]
    fn the_aided_fit_refuses_readings_that_do_not_follow_the_attitude() {
        let (a, b) = truth();
        let mut rng = Rng(12);
        let mut fit = AidedMagFit::default();
        for _ in 0..400 {
            let attitude = Quat::from_rotation_vector(rng.vector(1.5));
            let unrelated = a * (rng.direction() * STRENGTH) + b;
            assert!(fit.update(&state_at(attitude), &known(), unrelated, reference()));
        }
        assert_eq!(fit.solve(), None);
    }

    /// The field a chart with `error` of declination would give.
    fn charted(error: F) -> Vec3 {
        field_from_angles(0.05 + error, 1.0, STRENGTH)
    }

    #[test]
    fn a_level_vehicle_takes_a_declination_error_for_a_turned_sensor() {
        let (a, b) = truth();
        let wrong = charted(3.0_f64.to_radians());
        let mut fit = AidedMagFit::default();
        for k in 0..1000 {
            let attitude = Quat::from_euler(0.0, 0.0, k as F * 0.0123);
            let raw = a * attitude.rotate_inverse(reference()) + b;
            assert!(fit.update(&state_at(attitude), &known(), raw, wrong));
        }
        let found = fit.solve().expect("two axes seen");
        assert_eq!(found.observed_axes, 2);
        assert!(found.residual < 1e-6, "residual {}", found.residual);
        // The corrected reading is the one the filter will expect.
        for k in 0..50 {
            let attitude = Quat::from_euler(0.0, 0.0, k as F * 0.13);
            let raw = a * attitude.rotate_inverse(reference()) + b;
            let error = found.calibration.apply(raw) - attitude.rotate_inverse(wrong);
            assert!(error.norm() < 1e-6 * STRENGTH, "error {error:?}");
        }
    }

    #[test]
    fn a_tumbling_vehicle_shows_a_declination_error_in_the_residual() {
        let (a, b) = truth();
        let run = |degrees: F| {
            let wrong = charted(degrees.to_radians());
            let mut rng = Rng(13);
            let mut fit = AidedMagFit::default();
            for _ in 0..400 {
                let attitude = Quat::from_rotation_vector(rng.vector(1.5));
                let raw = a * attitude.rotate_inverse(reference()) + b;
                assert!(fit.update(&state_at(attitude), &known(), raw, wrong));
            }
            fit
        };
        let small = run(3.0).solve().expect("within the residual allowed");
        assert!(small.residual > 0.02, "residual {}", small.residual);
        assert_eq!(run(10.0).solve(), None);
    }

    #[test]
    fn the_strength_check_sees_a_changed_field() {
        let (a, b) = truth();
        let fit = sphere_fit(14, 0.1).solve().expect("a full sphere");
        let reading = fit
            .calibration
            .apply(a * (Vec3::new(0.6, 0.0, 0.8) * STRENGTH) + b);
        assert!(fit.is_plausible(reading, 0.05));
        assert!(!fit.is_plausible(reading * 1.2, 0.05));
        assert!(!fit.is_plausible(Vec3::splat(F::NAN), 0.05));
    }

    #[test]
    fn the_field_angles_point_where_they_say() {
        // North and down at no declination.
        let f = field_from_angles(0.0, core::f64::consts::FRAC_PI_4 as F, 2.0_f64.sqrt() as F);
        assert_relative_eq!(f.x, 1.0, epsilon = 1e-12);
        assert_relative_eq!(f.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(f.z, 1.0, epsilon = 1e-12);
        // East declination turns the horizontal part toward east.
        let f = field_from_angles(core::f64::consts::FRAC_PI_2 as F, 0.0, 1.0);
        assert_relative_eq!(f.y, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn the_identity_changes_nothing_and_a_singular_matrix_is_not_valid() {
        let v = Vec3::new(1.0, -2.0, 3.0);
        assert_eq!(MagCalibration::IDENTITY.apply(v), v);
        assert!(MagCalibration::IDENTITY.is_valid());
        let flat = MagCalibration {
            soft_iron: Vec3::new(1.0, 1.0, 0.0).to_diag(),
            ..MagCalibration::IDENTITY
        };
        assert!(!flat.is_valid());
        let not_a_number = MagCalibration {
            hard_iron: Vec3::splat(F::NAN),
            ..MagCalibration::IDENTITY
        };
        assert!(!not_a_number.is_valid());
    }

    #[test]
    fn the_eigen_solver_returns_a_right_handed_sorted_decomposition() {
        let mut rng = Rng(15);
        for _ in 0..200 {
            let q = Quat::from_rotation_vector(rng.vector(1.0)).to_dcm();
            let d = Vec3::new(rng.uniform(), rng.uniform() * 4.0, rng.uniform() * 9.0).to_diag();
            let m = q.matmul(&d).mul_transpose(&q);
            let (values, v) = symmetric_eigen(&m);
            assert!(values[0] <= values[1] && values[1] <= values[2]);
            assert_relative_eq!(determinant(&v), 1.0, epsilon = 1e-9);
            let rebuilt = v
                .matmul(&Vec3::from_array(values).to_diag())
                .mul_transpose(&v);
            assert!((rebuilt - m).amax() < 1e-9);
        }
    }
}
