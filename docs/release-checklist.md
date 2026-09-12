# Release checklist

Use this checklist before creating a PicoTracker release tag. The development version is recorded in [`../VERSION`](../VERSION).

## Source and regression gates

- [ ] `master` is clean and contains the intended release changes.
- [ ] `VERSION` is changed from the development suffix to the release version.
- [ ] `CHANGELOG.md` contains release notes for that exact version.
- [ ] GitHub Actions host tests pass on the release commit.
- [ ] `make -C tests clean test` passes locally.
- [ ] All IAR project source references resolve to tracked files.
- [ ] No release-blocking TODO, known infinite wait, or unresolved protocol error remains.

## Target build gates

- [ ] Open `firmware/HC12Tracker.ewp` with the supported IAR Embedded Workbench for STM8 toolchain.
- [ ] Perform a clean Debug build with zero errors.
- [ ] Perform a clean Release build with zero errors.
- [ ] Confirm the linked firmware fits within the STM8S003F3 8 KB flash limit and available RAM.
- [ ] Record the IAR version, build configuration, firmware size, and build date in the release notes.
- [ ] Export the release firmware artifact and calculate its SHA-256 checksum.

## Hardware gates

- [ ] Complete every required row in [`hardware-validation.md`](hardware-validation.md).
- [ ] Confirm GPS acquisition, degraded-mode telemetry, radio transmission, sleep/wake behavior, and power cycling on target hardware.
- [ ] Confirm the intended oscillator configuration and HC-12 radio variant for the release build.
- [ ] Retain representative decoded telemetry from the validation run.

## Release publication

- [ ] Review [`../LICENSE.md`](../LICENSE.md) and the standalone license files.
- [ ] Update README verification status if target validation is complete.
- [ ] Create an annotated `vX.Y.Z` tag only after all required gates pass.
- [ ] Publish release notes with build metadata, hardware configuration, known limitations, and artifact checksums.
- [ ] Attach the validated firmware artifact to the GitHub release.
