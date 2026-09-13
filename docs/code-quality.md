# Code quality checks

PicoTracker keeps the quality gate intentionally small and deterministic. The canonical command is:

```sh
make quality
```

It runs three checks:

- `cppcheck` on the maintained `firmware/` C sources with warning, performance, and portability diagnostics enabled
- Ruff linting and formatting verification for Python under `tools/` and `tests/`
- trailing-whitespace and final-newline validation for maintained text files

Host C regression tests are also compiled with `-Wall -Wextra -Werror`, so compiler warnings in host-testable firmware logic are already treated as failures.

## Python formatting

Ruff is pinned by the reproducible development container and configured in [`../pyproject.toml`](../pyproject.toml). To apply its safe lint fixes and formatter locally:

```sh
make format
```

`make quality` never modifies files; it only verifies them.

## Historical material

The whitespace gate deliberately excludes `hardware/`, `test_firmware/`, `cad/`, and the raw telemetry capture. Those paths contain preserved historical or binary/raw artifacts where mass cosmetic rewrites would obscure provenance without improving maintained code quality.

The production `firmware/` sources are included. Their pre-existing trailing whitespace was normalized when this gate was introduced; no functional C formatting rewrite was performed.
