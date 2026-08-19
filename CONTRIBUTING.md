# Contributing

PicoTracker is a small embedded hardware and firmware project with historical design material dating from the original Imperial College Space Society work.

## Good contributions

Focused changes are preferred, including:

- documentation corrections and clarification
- repository hygiene and build-file cleanup
- reproducible firmware fixes
- hardware notes backed by measurements or datasheets
- test procedures and recorded test results

## Before changing firmware or hardware

Please keep behavioural changes narrowly scoped and describe how they were tested. For hardware changes, include the affected component or revision and the evidence supporting the change.

## Repository hygiene

Do not commit generated IAR build output, editor metadata, local virtual environments, logs, or operating-system metadata. The repository `.gitignore` covers the common generated files used by this project.

## Pull requests

Keep pull requests small enough to review independently. Explain what changed, why it changed, and any testing performed. Historical files should be preserved where possible rather than rewritten without a technical reason.
