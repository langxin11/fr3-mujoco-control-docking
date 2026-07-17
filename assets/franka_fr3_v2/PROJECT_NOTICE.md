# Project modification notice

This directory is derived from the `franka_fr3_v2` model in Google DeepMind's
MuJoCo Menagerie. The upstream `LICENSE`, `README.md`, `CHANGELOG.md`, and mesh
assets are retained.

For this course project, `fr3v2.xml` replaces the upstream position servos with
torque actuators, adds an end-effector site, and sets reflected motor inertia for
the documented engineering-equivalent PMSM and reducer model. `scene.xml` adds a
target marker, report camera, and larger offscreen framebuffer. The Python
simulation computes joint torque through a field-oriented current loop rather
than treating the actuators as ideal position sources.

The PMSM electrical parameters and assumed reduction ratios are not official
Franka Research 3 internal drive parameters. See the project report for their
sources and the boundary of the engineering approximation.
