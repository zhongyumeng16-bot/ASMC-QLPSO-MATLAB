# ASMC-QLPSO MATLAB: essential source code

Essential numerical sources for *Asymptotic sliding mode control with chattering suppression and Q-reinforcement learning for nonlinear systems*.

## Run

Open MATLAB in the repository root, then run:

```matlab
addpath('MATLAB');
run_simulations;              % five methods, frozen parameters, 20 s each
benchmark_online_control;     % controller updates including internal integration
```

Requires base MATLAB; checked with MATLAB R2026a Update 3. Generated trajectories, metrics and timing samples go to `output/`. No online account or external dataset is required after downloading this repository.

Optional optimization (the full default study can take a long time):

```matlab
run_optimization(1,1003);       % one QLPSO run: 25 particles, 30 iterations
run_optimization;              % five methods, seeds 1001:1020
% build_accelerator;            % optional, requires MATLAB Coder
```

Task IDs: 1 ASMC-QLPSO; 2 ASMC-PSO; 3 ERLSMC-PSO; 4 PID-PSO; 5 saturation ERLSMC-PSO. Q learning and PSO run offline; the selected gains remain fixed during tracking.

## Essential files

| File | Purpose |
| --- | --- |
| `MATLAB/core/sr_model.m` | Two-joint dynamics and analytical model derivatives |
| `MATLAB/core/sr_reference.m` | Reference trajectory and derivatives |
| `MATLAB/core/sr_control.m` | ASMC, ERLSMC and PID control laws |
| `MATLAB/core/sr_balanced_trajectory.m` | Common-model simulation, RK4 and input integration; saturation ERLSMC implementation |
| `MATLAB/core/sr_balanced_metrics.m` | Objective evaluation and performance metrics |
| `MATLAB/core/evaluate_metrics.m` | Native MATLAB / optional local MEX dispatch |
| `MATLAB/core/sr_tune_balanced.m` | Q-learning-guided PSO and ordinary PSO |
| `MATLAB/run_optimization.m` | Seeds, budgets and optimization entry point |
| `MATLAB/run_simulations.m` | Simulations using the paper's frozen representative gains |
| `MATLAB/benchmark_online_control.m` | Execution-time measurement and controller-state consistency check |
| `MATLAB/legacy/legacy_*.m` | Historical ASMC/ERLSMC/PID numerical programs for Figures 4–6 |
| `reference_results/` | Only the fixed configuration, normalizers, weights and selected gains |
| `PARAMETERS.md` | Model assumptions, parameters and numerical conventions |

## Scope and provenance

This compact repository contains essential executable sources and necessary parameters. It excludes manuscript documents, Visio, pre-rendered figures, large archived output arrays and the full 20-seed result archive. Use the journal's `Supplementary_Code.zip` for the complete figure-reproduction and archived-result package.

The revised shared-model simulations support Figure 7 and the revised comparison protocol. Historical Figures 4–6 come from a separate implementation; their original arithmetic is retained in `legacy/`, not relabeled as a uniform-model comparison. To run those programs directly:

```matlab
addpath('MATLAB/legacy');
asmc = legacy_asmc();
erlsmc = legacy_erlsmc();
pid = legacy_pid();
```

The default driver does not retune parameters. Selected-run metrics should not be confused with Table 5's medians over 20 independent tuning runs. Parameter selection and the known historical implementation differences are documented in `PARAMETERS.md`.

Timing includes the numerical controller update and internal integration; it excludes plant integration, sensing/estimation, communications and actuator I/O. Desktop timings are not hard real-time guarantees. Use `benchmark_online_control(true)` to check controller updates without running the timer. Rerun times vary with the host; the publication's recorded timing samples remain in the full journal supplement.

## Manuscript version

Prepared for the November 2026 minor revision of TJCD-2026-0663.R1. Paired manuscript: `Manuscript_R2_Anonymous_Blue.docx` (MathType revision, prepared 2026-11-08). SHA-256: `00520e1338a9bc6c950714648d0adcfe23e3fa71aa9954f145c981d2edaad2b4`. The manuscript itself is not included in this source repository.

## Origin of the implementations

The `legacy_*` functions were extracted from the supplied mechanical-arm programs `arm_SMC_ASMC.m`, `arm_SMC_VSC.m` and `arm_PID.m`. They are portable adaptations, not byte-for-byte copies: script side effects were removed, and the missing ASMC helper was specialized to the active linear case. The exact source hashes and these changes are recorded in the full journal supplement.

The common-model `sr_*` implementations, 20-seed comparison, timing benchmark and portable entry points were added during manuscript revision. They are not represented as files already present in the original supplied archive. The current audit recomputed the metrics for all 100 saved parameter sets and matched the saved results; it did not repeat all 100 optimizer searches. The full journal supplement contains the 100-run record and verification results. This compact repository contains the algorithms needed to rerun that study and the selected representative parameters, but not the full saved 100-run archive.
