# First project: trace a failed telemetry transmission

This tutorial is a hardware-free practice project for a first PicoTracker contribution. It takes one small change through the same loop used for real work: reproduce behaviour, edit code, add a regression test, run the project gates, inspect the diff, and prepare a pull request.

The exercise adds a named `radio-loss` simulator scenario. It is intentionally small and should be treated as practice rather than submitted as a pull request; use the same workflow on an open issue for a real contribution.

## What you will learn

By the end you will have:

- run real captured telemetry through the decoder;
- reproduced a simulated radio-transmit failure;
- changed one Python control path;
- added a regression test for that change;
- generated and decoded a synthetic telemetry capture;
- run the same checks used by CI; and
- inspected the exact diff a maintainer would review.

No STM8 board, GPS receiver, radio, IAR installation, or RF equipment is required.

## 1. Prepare a clean branch

For a real contribution, first fork PicoTracker on GitHub, then clone your fork and add the main repository as `upstream`:

```sh
git clone https://github.com/YOUR-USERNAME/PicoTracker.git
cd PicoTracker
git remote add upstream https://github.com/ImperialSpaceSociety/PicoTracker.git
git switch -c tutorial/radio-loss-scenario
make demo
make test
```

Replace `YOUR-USERNAME` with your GitHub login. If you only want to do the exercise locally, you may instead clone `ImperialSpaceSociety/PicoTracker` directly and skip the `upstream` remote.

`make demo` should report **165 valid frames**. `make test` should complete without failures before you change anything; that gives you a known-good baseline.

If you use the repository Dev Container, the complete compiler, SDCC, cppcheck, and Ruff toolchain is already available. See [`development-environment.md`](development-environment.md).

## 2. Reproduce the behaviour first

The simulator already lets you inject a one-off transmit failure. Run 14 cycles, fail transmission on cycle 12, and write the successfully transmitted frames to a temporary capture:

```sh
make simulate SIM_ARGS="--cycles 14 --tx-fail 12 --capture /tmp/picotracker-tutorial.txt"
```

Near the end of the output you should see cycle 12 marked `FAILED`, followed by:

```text
Summary: cycles=14 degraded=0 measurement_failures=0 tx_failures=1 double_sleep=10 sleep_time=737.280s
Wrote 13 transmitted frames to /tmp/picotracker-tutorial.txt
```

Now decode that generated capture with the normal decoder:

```sh
make decode CAPTURE=/tmp/picotracker-tutorial.txt
```

Expected result:

```text
/tmp/picotracker-tutorial.txt: 13 valid frames
```

This is the important end-to-end observation: the simulator completed 14 tracker cycles, but only 13 frames reached the synthetic ground capture because cycle 12 failed to transmit.

## 3. Add a named scenario

Open [`tools/simulate_tracker.py`](../tools/simulate_tracker.py). In `scenario_faults()`, add a named scenario that produces the same transmit failure:

```python
if name == "radio-loss":
    return set(), set(), {12}
```

Then add `"radio-loss"` to the `--scenario` choices in `main()`.

The three sets returned by `scenario_faults()` are, in order, GPS-loss cycles, measurement-failure cycles, and transmit-failure cycles. This scenario therefore changes only transmission behaviour.

Do not change the telemetry format or embedded firmware for this exercise.

## 4. Add the regression test

Open [`tests/test_simulate_tracker.py`](../tests/test_simulate_tracker.py) and add a focused test inside `TrackerSimulatorTests`:

```python
def test_named_radio_loss_scenario(self):
    gps_loss, measurement_fail, tx_fail = sim.scenario_faults("radio-loss")
    self.assertEqual(gps_loss, set())
    self.assertEqual(measurement_fail, set())
    self.assertEqual(tx_fail, {12})

    results = sim.run_simulation(
        sim.synthetic_profile()[:14],
        tx_fail_cycles=tx_fail,
    )
    self.assertFalse(results[11].tx_success)
```

`results[11]` is cycle 12 because Python lists are zero-indexed. The test checks both pieces of the contract: the named scenario selects only cycle 12 for transmit failure, and the simulator actually reports that cycle as not transmitted.

Avoid testing unrelated behaviour in the same test. Small tests are easier to diagnose and easier for a maintainer to review.

## 5. Format and test the change

Run:

```sh
make format
make test
```

Then exercise the new named path:

```sh
make simulate SIM_ARGS="--scenario radio-loss --cycles 14 --capture /tmp/picotracker-tutorial.txt"
make decode CAPTURE=/tmp/picotracker-tutorial.txt
```

You should again see one transmit failure and **13 valid frames** in the capture.

For the complete repository gate, run either:

```sh
make check
```

or, with Docker:

```sh
make container-check
```

`make check` includes static analysis, Python quality checks, host regression tests, decoder and simulator smoke tests, and the SDCC structural STM8 build. The structural SDCC image is a compile/link and memory-window check; it is not flashable firmware.

## 6. Review your own diff

Before asking someone else to review a change, inspect exactly what changed:

```sh
git status --short
git diff -- tools/simulate_tracker.py tests/test_simulate_tracker.py
```

The output of `git diff` is essentially the same change view GitHub shows in a pull request under **Files changed**. Green lines are additions and red lines are removals.

For this exercise, the intended diff is only the simulator scenario and its regression test. Generated captures under `/tmp` should not appear in Git at all.

## 7. Understand the system connection

The practice change is in the hardware-independent simulator, but it models a real tracker boundary:

1. the tracker acquires or retains a navigation fix;
2. it builds a CRC-protected telemetry frame;
3. the radio transmission either succeeds or fails;
4. only successfully transmitted frames can appear in a ground capture; and
5. the decoder validates the frames that were actually received.

That is why the exercise uses both the simulator and the decoder rather than stopping at a unit test. You are checking behaviour across two project layers.

For the embedded implementation, follow [`firmware/main.c`](../firmware/main.c) into [`firmware/telemetry.c`](../firmware/telemetry.c) and [`firmware/si_trx.c`](../firmware/si_trx.c). Do not change those files merely to complete this tutorial.

## 8. Apply the workflow to a real contribution

Do **not** open a pull request containing the exact `radio-loss` tutorial change. Restore the practice edits when you are finished:

```sh
git restore tools/simulate_tracker.py tests/test_simulate_tracker.py
```

Then choose an open issue or another agreed task and use the same loop:

**reproduce → make one focused change → add evidence/tests → run checks → inspect the diff → open a pull request**.

For a real task, create a fresh branch from current `master`, commit only the relevant files, and push that branch to your fork:

```sh
git switch master
git pull --ff-only upstream master
git switch -c feature/short-description
git add path/to/changed-file path/to/test-file
git commit -m "type: concise description"
git push -u origin feature/short-description
```

Open the pull request against `ImperialSpaceSociety/PicoTracker:master`. In the PR description, state what changed, why it changed, and exactly how you validated it. The repository will request the code owner's review automatically and the required CI check must pass before merge.

If a maintainer requests changes, update the same branch and push again; the existing pull request updates automatically. Do not open a replacement PR for every revision.

## Next steps

After this tutorial, choose a path in [`getting-started.md`](getting-started.md), inspect the open issues, or use [`roadmap.md`](roadmap.md) to understand larger project directions. For contribution rules and review expectations, see [`../CONTRIBUTING.md`](../CONTRIBUTING.md).
