//! Between the filter's coordinates and physical ones.
//!
//! [`EqFilter`](crate::filter::EqFilter)'s covariance is over `ε`, the normal
//! coordinates the symmetry induces. `ε` is not the error anyone means by
//! "position error" or "attitude error", and the difference is not small:
//! `ε₁,ω` rotates the whole trajectory about the **global origin**, so a
//! position error is `−p̂^ ε₁,ω + ε₁,ρ`, two large and strongly anti-correlated
//! terms. Two kilometres from the anchor the position block of `Σ` can grow
//! while the position is known to centimetres.
//!
//! So anything that states an uncertainty in metres, or is handed one, has to
//! cross between the two, and it has to cross in both directions:
//!
//! - **out**, to report: [`covariance`] and [`variances`] map `Σ` to physical
//!   errors.
//! - **in**, to initialise: [`covariance_from_std`] maps independent physical
//!   sigmas to the `Σ` that describes them.
//!
//! The second is the easy one to get wrong. Drawing initial errors
//! independently in physical coordinates and handing the filter `diag(σ²)`
//! treats the two charts as the same, which they are only at the anchor with no
//! velocity. A Monte-Carlo harness that does this reads the filter 12 %
//! overconfident with nothing in the filter at fault. An engine accepting
//! physical sigmas, the only kind anyone has, goes through this module.
//!
//! # The map
//!
//! With `X̂` the observer state, `ξ̂ = φ(X̂, ξ°)` the estimate, and the truth at
//! normal coordinates `ε` about it, `ξ = φ(exp(ε) X̂, ξ°)`, the physical error
//! `ξ ⊖ ξ̂` is to first order
//!
//! ```text
//! δφ = ε₁,ω                     δb = −Ad_{B̂⁻¹} ε₂
//! δv = −v̂^ ε₁,ω + ε₁,ν          δt = −Âᵀ ε₃
//! δp = −p̂^ ε₁,ω + ε₁,ρ          δS = Êᵀ(ε₄ − ε₁,ω)
//! ```
//!
//! [`jacobian`] is that matrix and `the_jacobian_matches_a_numerical_one`
//! differentiates the group action to check it, at an observer state with no
//! zero blocks. It is block lower triangular, so its inverse is written down
//! rather than computed:
//!
//! ```text
//! ε₁,ω = δφ                     ε₂ = −Ad_B̂ δb
//! ε₁,ν = δv + v̂^ δφ             ε₃ = −Â δt
//! ε₁,ρ = δp + p̂^ δφ             ε₄ = Ê δS + δφ
//! ```
//!
//! # In a rotating frame
//!
//! The group's velocity there is `v' = v + ω×p`
//! ([`GlobalFrame`](crate::lift::GlobalFrame)), and what anyone means by a
//! velocity error is the one over the ground, `δv = δv' − ω^ δp`. So the
//! velocity row becomes
//!
//! ```text
//! δv = (ω^ p̂^ − v̂'^) ε₁,ω + ε₁,ν − ω^ ε₁,ρ
//! ε₁,ν = δv + ω^ δp + v̂'^ δφ
//! ```
//!
//! which is why every map here takes `earth_rate`. Pass zero for a flat
//! frame.
//!
//! # Error convention
//!
//! [`error`] is **truth minus estimate**, with the two rotations as the
//! rotation vectors of `R R̂ᵀ` and `Ŝᵀ S`. Attitude is therefore a rotation
//! about the global axes — north, east and down in this workspace — which is
//! the convention the ESKF's attitude error uses too. It compares two [`State`]s
//! as they stand, so in a rotating frame its velocity entry is `δv'`, which is
//! the one to set against [`jacobian`] with a zero `earth_rate`.

use drifters_core::math::{Mat3, Matrix, Quat, Vec3};
use drifters_core::F;

use crate::group::{State, Symmetry};
use crate::linear::{StateMatrix, DIM};

/// Row of the attitude block in physical coordinates, and in `ε`.
pub const ATTITUDE: usize = 0;
/// Row of the velocity block.
pub const VELOCITY: usize = 3;
/// Row of the position block.
pub const POSITION: usize = 6;
/// Row of the gyroscope-bias block.
pub const GYRO_BIAS: usize = 9;
/// Row of the accelerometer-bias block.
pub const ACCEL_BIAS: usize = 12;
/// Row of the GNSS lever-arm block.
pub const LEVER: usize = 15;
/// Row of the magnetometer-calibration block.
pub const MAG: usize = 18;

/// `∂(physical error) / ∂ε` at the observer state `x`, in a frame turning at
/// `earth_rate` (zero for a flat one).
///
/// See the [module documentation](self) for the relations and their order.
pub fn jacobian(x: &Symmetry, earth_rate: Vec3) -> StateMatrix {
    let mut j = StateMatrix::zeros();
    // φ(X̂, ξ°) has pose Ĉ, so the estimate's velocity and position are Ĉ's
    // own columns.
    let (v, p) = (x.c.velocity, x.c.position);
    let w = earth_rate.skew();

    j.set_block(ATTITUDE, 0, &Mat3::identity());
    j.set_block(VELOCITY, 0, &(w.matmul(&p.skew()) - v.skew()));
    j.set_block(VELOCITY, 3, &Mat3::identity());
    j.set_block(VELOCITY, 6, &-w);
    j.set_block(POSITION, 0, &-p.skew());
    j.set_block(POSITION, 6, &Mat3::identity());
    j.set_block(GYRO_BIAS, 9, &-x.b().inverse().adjoint());
    j.set_block(LEVER, 15, &-x.a().transpose());
    let et = x.e.transpose();
    j.set_block(MAG, 0, &-et);
    j.set_block(MAG, 18, &et);
    j
}

/// `∂ε / ∂(physical error)` at `x`: the inverse of [`jacobian`].
pub fn jacobian_inverse(x: &Symmetry, earth_rate: Vec3) -> StateMatrix {
    let mut m = StateMatrix::zeros();
    let (v, p) = (x.c.velocity, x.c.position);

    m.set_block(0, ATTITUDE, &Mat3::identity());
    m.set_block(3, ATTITUDE, &v.skew());
    m.set_block(3, VELOCITY, &Mat3::identity());
    m.set_block(3, POSITION, &earth_rate.skew());
    m.set_block(6, ATTITUDE, &p.skew());
    m.set_block(6, POSITION, &Mat3::identity());
    m.set_block(9, GYRO_BIAS, &-x.b().adjoint());
    m.set_block(15, LEVER, &-x.a());
    m.set_block(18, ATTITUDE, &Mat3::identity());
    m.set_block(18, MAG, &x.e);
    m
}

/// The physical error `truth − estimate`, in block order.
pub fn error(truth: &State, estimate: &State) -> [F; DIM] {
    let mut e = [0.0; DIM];
    let attitude = Quat::from_dcm(
        &truth
            .pose
            .rotation
            .matmul(&estimate.pose.rotation.transpose()),
    )
    .to_rotation_vector();
    let mag = Quat::from_dcm(&estimate.mag.transpose().matmul(&truth.mag)).to_rotation_vector();
    let bias = truth.bias - estimate.bias;
    let blocks = [
        attitude,
        truth.pose.velocity - estimate.pose.velocity,
        truth.pose.position - estimate.pose.position,
        bias.omega,
        bias.nu,
        truth.lever - estimate.lever,
        mag,
    ];
    for (chunk, block) in e.chunks_exact_mut(3).zip(blocks) {
        chunk.copy_from_slice(&block.to_array());
    }
    e
}

/// `J Σ Jᵀ`: the covariance of the physical error.
///
/// Two `21 × 21` temporaries. A diagnostic, not something to call per sample;
/// [`variances`] is the lighter question and usually the one being asked.
pub fn covariance(x: &Symmetry, earth_rate: Vec3, sigma: &StateMatrix) -> StateMatrix {
    let j = jacobian(x, earth_rate);
    let mut j_sigma = StateMatrix::zeros();
    j.matmul_into(sigma, &mut j_sigma);
    let mut out = StateMatrix::zeros();
    j_sigma.mul_transpose_into(&j, &mut out);
    out
}

/// The diagonal of `J Σ Jᵀ`: the variance of each physical error.
///
/// Row by row, so the product is never formed and the only temporary is `J`.
pub fn variances(x: &Symmetry, earth_rate: Vec3, sigma: &StateMatrix) -> [F; DIM] {
    let j = jacobian(x, earth_rate);
    let mut out = [0.0; DIM];
    for (variance, row) in out.iter_mut().zip(j.data.iter()) {
        let mut acc = 0.0;
        for (a, sigma_row) in row.iter().zip(sigma.data.iter()) {
            if *a == 0.0 {
                // J is nine-tenths zeros.
                continue;
            }
            let mut inner = 0.0;
            for (s, b) in sigma_row.iter().zip(row.iter()) {
                inner += s * b;
            }
            acc += a * inner;
        }
        *variance = acc;
    }
    out
}

/// The `Σ` that describes physical errors drawn **independently**, with
/// standard deviations `std`, about the estimate `x` represents.
///
/// `std` is in block order: attitude (rad), velocity, position, gyroscope
/// bias, accelerometer bias, lever arm, magnetometer calibration (rad).
///
/// `M diag(σ²) Mᵀ` with `M` the [`jacobian_inverse`]. Not diagonal, and not
/// meant to be: an attitude error at a distance *is* a position error in `ε`,
/// and a filter told otherwise believes it has information it was never given.
pub fn covariance_from_std(x: &Symmetry, earth_rate: Vec3, std: &[F; DIM]) -> StateMatrix {
    let m = jacobian_inverse(x, earth_rate);
    // M diag(σ²), by scaling M's columns.
    let mut scaled = m;
    for row in scaled.data.iter_mut() {
        for (value, s) in row.iter_mut().zip(std.iter()) {
            *value *= s * s;
        }
    }
    let mut out = StateMatrix::zeros();
    scaled.mul_transpose_into(&m, &mut out);
    out.symmetrize();
    out
}

/// A `3 × 3` block of `J Σ Jᵀ` on its own, for the rows starting at `row`.
///
/// What a caller wanting one quantity's uncertainty needs, at three rows of
/// `J` rather than all twenty-one.
pub fn block_covariance(x: &Symmetry, earth_rate: Vec3, sigma: &StateMatrix, row: usize) -> Mat3 {
    let j = jacobian(x, earth_rate);
    let mut rows = Matrix::<3, DIM>::zeros();
    for (out, source) in rows.data.iter_mut().zip(j.data.iter().skip(row)) {
        *out = *source;
    }
    rows.matmul(sigma).mul_transpose(&rows)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    use crate::group::{act_state, Algebra};
    use crate::lie::{Se23, Se3Tangent};
    use crate::lift::GlobalFrame;

    /// Far above the Earth's, so that a missing term cannot pass as round-off.
    const SPIN: Vec3 = Vec3::new(0.012, -0.007, 0.021);

    fn rot(x: F, y: F, z: F) -> Mat3 {
        Quat::from_rotation_vector(Vec3::new(x, y, z)).to_dcm()
    }

    /// An estimate with nothing zero in it: moving, away from the anchor,
    /// tilted, with a bias, a lever arm and a magnetometer calibration. Every
    /// off-diagonal block of the Jacobian is exercised, which the identity
    /// would not do — see docs/eqf.md on what a Jacobian checked at the
    /// identity can hide.
    fn estimate() -> State {
        State {
            pose: Se23::new(
                rot(0.3, -0.2, 0.9),
                Vec3::new(11.0, -4.0, 0.6),
                Vec3::new(420.0, -310.0, -18.0),
            ),
            bias: Se3Tangent::new(
                Vec3::new(2.0e-3, -1.5e-3, 3.0e-3),
                Vec3::new(0.03, -0.02, 0.04),
            ),
            lever: Vec3::new(0.35, -0.12, -0.9),
            mag: rot(0.12, -0.08, 0.05),
        }
    }

    /// The Jacobian against a central difference of the thing it linearises:
    /// the physical error of `φ(exp(ε) X̂, ξ°)` from `φ(X̂, ξ°)`.
    #[test]
    fn the_jacobian_matches_a_numerical_one() {
        let x = Symmetry::from_state(&estimate());
        let here = act_state(&x, &State::default());
        let analytic = jacobian(&x, Vec3::ZERO);

        // Large enough that the difference keeps its digits against a 420 m
        // position, small enough that the second-order term stays below them.
        let h = 1.0e-6;
        for k in 0..DIM {
            let mut basis = [0.0; DIM];
            basis[k] = h;
            let step = Algebra::from_array(&basis);
            let ahead = act_state(&Symmetry::exp(step).compose(&x), &State::default());
            let behind = act_state(
                &Symmetry::exp(step.scaled(-1.0)).compose(&x),
                &State::default(),
            );
            let plus = error(&ahead, &here);
            let minus = error(&behind, &here);
            for i in 0..DIM {
                let numeric = (plus[i] - minus[i]) / (2.0 * h);
                assert_relative_eq!(analytic[(i, k)], numeric, epsilon = 1.0e-6);
            }
        }
    }

    #[test]
    fn the_inverse_is_the_inverse() {
        let x = Symmetry::from_state(&estimate());
        let identity = StateMatrix::identity();
        for rate in [Vec3::ZERO, SPIN] {
            let product = jacobian(&x, rate).matmul(&jacobian_inverse(&x, rate));
            for i in 0..DIM {
                for k in 0..DIM {
                    assert_relative_eq!(product[(i, k)], identity[(i, k)], epsilon = 1.0e-12);
                }
            }
        }
    }

    #[test]
    fn the_jacobian_is_the_identity_only_at_rest_at_the_anchor() {
        let at_rest = Symmetry::IDENTITY;
        let j = jacobian(&at_rest, Vec3::ZERO);
        // Up to the signs the bias and lever blocks carry by convention, and
        // the magnetometer's coupling to attitude, which is never absent.
        for i in 0..9 {
            for k in 0..DIM {
                assert_eq!(j[(i, k)], if i == k { 1.0 } else { 0.0 });
            }
        }
        let moved = jacobian(&Symmetry::from_state(&estimate()), Vec3::ZERO);
        assert!(
            moved.block::<3, 3>(POSITION, 0).amax() > 100.0,
            "away from the anchor, attitude must leak into position"
        );
    }

    /// The property the prior exists for: mapped out again, it is the
    /// independent sigmas it was built from.
    #[test]
    fn a_prior_from_physical_sigmas_reads_back_as_those_sigmas() {
        let x = Symmetry::from_state(&estimate());
        let mut std = [0.0; DIM];
        // Deliberately not isotropic within a block: a rotation leaves an
        // isotropic block alone, and would hide a missing Â or Ê.
        for (i, s) in std.iter_mut().enumerate() {
            let scale = match i / 3 {
                0 => 0.02,
                1 => 0.2,
                2 => 1.0,
                3 => 5.0e-4,
                4 => 5.0e-3,
                5 => 0.05,
                _ => 0.02,
            };
            *s = scale * (1.0 + 0.5 * (i % 3) as F);
        }
        for rate in [Vec3::ZERO, SPIN] {
            let sigma = covariance_from_std(&x, rate, &std);
            let back = covariance(&x, rate, &sigma);
            for i in 0..DIM {
                for k in 0..DIM {
                    let want = if i == k { std[i] * std[i] } else { 0.0 };
                    let scale = std[i] * std[k];
                    assert!(
                        (back[(i, k)] - want).abs() < 1.0e-9 * scale,
                        "physical covariance ({i}, {k}) is {} against {want}",
                        back[(i, k)]
                    );
                }
            }
            let diagonal = variances(&x, rate, &sigma);
            for i in 0..DIM {
                assert_relative_eq!(diagonal[i], std[i] * std[i], max_relative = 1.0e-9);
            }
            let position = block_covariance(&x, rate, &sigma, POSITION);
            for i in 0..3 {
                assert_relative_eq!(
                    position[(i, i)],
                    std[POSITION + i] * std[POSITION + i],
                    max_relative = 1.0e-9
                );
            }
        }
    }

    /// The velocity row in a rotating frame, against a central difference of
    /// the velocity over the ground. The other rows do not involve the rate,
    /// and `the_jacobian_matches_a_numerical_one` holds them.
    #[test]
    fn the_velocity_row_is_the_velocity_over_the_ground() {
        let frame = GlobalFrame::rotating(Vec3::new(0.0, 0.0, 9.81), SPIN);
        let x = Symmetry::from_state(&estimate());
        let analytic = jacobian(&x, SPIN);
        let ground = |s: &State| frame.ground_velocity(s.pose.velocity, s.pose.position);

        let h = 1.0e-6;
        for k in 0..DIM {
            let mut basis = [0.0; DIM];
            basis[k] = h;
            let step = Algebra::from_array(&basis);
            let ahead = act_state(&Symmetry::exp(step).compose(&x), &State::default());
            let behind = act_state(
                &Symmetry::exp(step.scaled(-1.0)).compose(&x),
                &State::default(),
            );
            let numeric = (ground(&ahead) - ground(&behind)) * (0.5 / h);
            for i in 0..3 {
                assert_relative_eq!(analytic[(VELOCITY + i, k)], numeric[i], epsilon = 1.0e-6);
            }
        }
    }

    /// And the reason it has to be built: in `ε` it is not diagonal at all.
    #[test]
    fn that_prior_is_not_diagonal_in_the_filters_coordinates() {
        let x = Symmetry::from_state(&estimate());
        let mut std = [0.0; DIM];
        for (i, s) in std.iter_mut().enumerate() {
            *s = if i < 3 { 0.02 } else { 1.0e-3 };
        }
        let sigma = covariance_from_std(&x, Vec3::ZERO, &std);
        // 0.02 rad of attitude at |p̂| = 522 m is ten metres of ε-position.
        let position_variance = sigma[(6, 6)] + sigma[(7, 7)] + sigma[(8, 8)];
        assert!(
            position_variance > 50.0,
            "ε-position variance {position_variance} must carry the attitude prior"
        );
        assert!(
            sigma[(6, 1)].abs() > 1.0e-3,
            "and be correlated with it: Σ(6, 1) = {}",
            sigma[(6, 1)]
        );
        assert!(sigma.asymmetry() < 1.0e-15);
        assert!(drifters_core::math::Cholesky::new(&sigma).is_some());
    }

    #[test]
    fn an_exact_estimate_has_no_error() {
        let s = estimate();
        for value in error(&s, &s) {
            assert_eq!(value, 0.0);
        }
    }
}
