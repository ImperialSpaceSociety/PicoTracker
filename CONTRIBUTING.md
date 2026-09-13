# Contributing

PicoTracker is a small embedded hardware and firmware project with historical design material dating from the original Imperial College Space Society work.


## New contributors

If this is your first time in the repository, start with [`docs/getting-started.md`](docs/getting-started.md). It gives you a hardware-free first run, explains the main subsystems, and suggests contribution paths by interest. Open tasks labelled [`good first issue`](https://github.com/ImperialSpaceSociety/PicoTracker/issues?q=is%3Aissue%20is%3Aopen%20label%3A%22good%20first%20issue%22) are intended to be narrow enough to enter without prior PicoTracker knowledge.

You do not need IAR or physical tracker hardware for documentation, decoder, host-test, protocol-test, and many tooling contributions.

## Good contributions

Focused changes are preferred, including:

- documentation corrections and clarification
- repository hygiene and build-file cleanup
- reproducible firmware fixes
- hardware notes backed by measurements or datasheets
- test procedures and recorded test results

## Before changing firmware or hardware

Please keep behavioural changes narrowly scoped and describe how they were tested. For hardware changes, include the affected component or revision and the evidence supporting the change.

Before submitting a firmware or host-tool change, run `make quality` and `make test`. For the complete gate, run `make check`; if you prefer the reproducible containerized toolchain, run `make container-check`. Changes intended for a release must also satisfy the gates in [`docs/release-checklist.md`](docs/release-checklist.md).

## Repository hygiene

Do not commit generated IAR build output, editor metadata, local virtual environments, logs, or operating-system metadata. The repository `.gitignore` covers the common generated files used by this project.

## Pull requests

Keep pull requests small enough to review independently. Explain what changed, why it changed, and any testing performed. Historical files should be preserved where possible rather than rewritten without a technical reason.
