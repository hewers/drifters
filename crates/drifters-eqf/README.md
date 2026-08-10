# drifters-eqf

Equivariant filter (EqF) for inertial navigation.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std`
GNSS/INS sensor fusion library.

An implementation of Fornasier et al., *An Equivariant Approach to Robust State
Estimation for the ArduPilot Autopilot System* (ICRA 2024), on the theory of
van Goor, Hamel and Mahony (arXiv:2010.14666).

**Work in progress.** The Lie group machinery — SE_2(3), se(3)/se_2(3),
exp/log, adjoints — is implemented and tested against the defining identities.
The filter itself is not yet built.

## Licence

MIT OR Apache-2.0, at your option.
