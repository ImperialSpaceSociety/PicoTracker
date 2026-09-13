# Reproducible development environment

PicoTracker includes a containerized development environment so contributors can use the same Linux toolchain exercised by CI without installing GCC or SDCC directly on the host.

## Dev Container

With a Dev Container-compatible editor, open the repository and choose **Reopen in Container**. The configuration in [`.devcontainer/`](../.devcontainer/) installs Python 3.12, GCC, Make, Git, and SDCC and runs `make demo` after the workspace is created.

Once the container is ready:

```sh
make test
make stm8
make check
```

## Plain Docker

Docker users do not need an editor integration:

```sh
make container-build
make container-check
```

`make container-check` builds the development image and runs the complete `make check` verification inside it. Override the local image name with `DEV_IMAGE=...` if required.

## What the container does not provide

The container intentionally excludes the proprietary IAR STM8 toolchain and physical programming/debug hardware. It validates host-testable firmware logic, telemetry tooling, and the SDCC structural STM8 build; release hardware validation and a native IAR build remain separate activities.
