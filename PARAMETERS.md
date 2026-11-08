# Model, controller and optimization parameters

## Common physical values

| Parameter | Value |
| --- | --- |
| m1, m2 | 0.3, 0.5 kg |
| l1, l2 | 0.2, 0.3 m |
| J1, J2 | 0.05, 0.1 kg m^2 |
| g | 9.81 m/s^2 |
| Initial joint positions | [0.5; 0.55] rad |
| Initial velocities | [0; 0] rad/s |
| Reference positions | [sin(2t); sin(0.8t)] rad |
| Simulation duration / controller period | 20 s / 0.001 s |

The baseline disturbance vector is

```text
d1(t) = 15*(0.2*sin(2*t)+0.1*sin(4*t)+0.05*cos(1.5*t))
d2(t) = 15*(0.02*cos(1.5*t)+0.3*sin(2*t)+0.05*cos(0.5*t))
```

## Historical simulations: Figures 4-6

The standalone `legacy_*` functions contain their numerical model, controller, disturbance and stepping rules. They preserve the active arithmetic of the supplied mechanical-arm scripts, with no retuning.

| Controller | Fixed parameters |
| --- | --- |
| ASMC | alpha=[23.5652;24.036504], beta=[2.1023;2.0896862], eta=[0.1292;0.133076], rho=120, sign switching, P=Q=1 |
| ERLSMC | c=2.0928*I, K=-16.4462*I, switching coefficient=60.9933*I; compensation values dH=2, dL=-2 |
| PID | Kp=[100;100], Ki=[5;10], Kd=[4;2]; backward-difference error derivative |

These scripts have historical implementation details that should be kept distinct from the revised shared model: the ASMC velocity step uses `pinv(H)*ddq`; the ERLSMC matrix calculations retain their original scalar-state updates and `c*dq` term, with the original negative K; the PID plant equation includes the disturbance vector twice. Those arithmetic choices are preserved for reproducing the actual archived figures, not silently corrected. They must not be interpreted as a newly established uniform-model comparison. The revised functions below use common canonical dynamics and a single matched disturbance vector.

The original missing `kaifang` dependency is removed only for the active linear P=Q=1 case: the surface term is `alpha*e` and its derivative contribution is `alpha*ed`. No alternative nonlinear exponent implementation is claimed. CATE for Figure 6 is `cumsum(abs(error))`, whereas the revised IAE uses time integration. Figure 5 sets the first PID input point to zero for display, following the original plotting script; the underlying saved/recomputed input array is preserved.

## Revised shared model and solvers: Figure 7 and revised metrics

The dynamics are `H(q)*qddot + h(q,qdot) = u + d(t)`, with the inertia matrix, Coriolis/centrifugal terms, gravity and analytical model derivatives defined in `sr_model.m`. All revised methods use this common model and disturbance. Model parameters and derivatives are known nominal quantities; state, velocity and ASMC acceleration feedback are ideal simulation signals. No observer or unknown-input-gain estimator is implemented. Actuator torque limit is Inf in the frozen protocol.

At each controller sample, commands are updated once. The plant is advanced by RK4 with `cfg(16)=1` substep in the frozen protocol; an ASMC torque-rate command is integrated as a linear torque ramp during those stages. Direct torque commands for ERLSMC/PID are held over the sample. PID error integration is trapezoidal. The frozen configuration vector is stored in `frozen_normalization.mat`:

```text
cfg(1:7)  = [m1,m2,l1,l2,J1,J2,g]
cfg(8:9)  = reference frequencies
cfg(10)   = disturbance waveform selector (0: baseline)
cfg(11)   = disturbance amplitude (15)
cfg(12:13)= initial positions
cfg(14:17)= [duration, controller period, RK4 substeps, torque limit]
```

| Method / kind | Parameter ordering | Switching |
| --- | --- | --- |
| ASMC-QLPSO and ASMC-PSO / 0 | [alpha1,alpha2,beta1,beta2,eta1,eta2] | rho=120, sat(s/0.5) |
| ERLSMC-PSO / 1 | [lambda1,lambda2,K1,K2,R1,R2] | sign(s) |
| PID-PSO / 2 | [Kp1,Kp2,Ki1,Ki2,Kd1,Kd2] | Direct PID torque |
| ERLSMC-SAT-PSO / 3 | [lambda1,lambda2,K1,K2,R1,R2,width] | Tuned saturation width |

ASMC has `s=alpha.*e+beta.*ed+eta.*(rdd-acc)` and a nominal derivative-based torque-rate law. ERLSMC has `s=ed+lambda.*e`. `selected_parameters.json` gives the exact representative gains for all five methods; ASMC-QLPSO seed 1003 and ERLSMC-PSO seed 1004 generate Figure 7. Representatives are runs nearest to the within-method median training J.

The trajectory matrix has columns: time(1), position(2:3), velocity(4:5), acceleration(6:7), actual torque(8:9), native surface(10:11), tracking error(12:13), torque rate(14:15), saturation flags(16:17).

## Performance and optimization protocol

For each joint, IAE=`integral(abs(e))`, ITAE=`integral(t*abs(e))`, ISU=`integral(u^2)`, TV=`sum(abs(diff(u)))`. Integrals use trapezoidal quadrature. The optimizer aggregates IAE, ITAE, ISU and TV across joints, and uses pooled two-joint RMSE. The compact entry point writes aggregate representative-run metrics to `output/recomputed_representative_metrics.csv`. Representative runs are not the 20-seed medians in Table 5; the full journal supplement retains those archived results.

```text
J = 0.30*IAE/nIAE + 0.20*RMSE/nRMSE + 0.20*ISU/nISU + 0.30*TV/nTV
[nIAE,nRMSE,nISU,nTV] =
[1.3415413961970373,0.044262268121982293,757.52660548287417,500.26265868546068]
F = J/(1+J)
```

Normalizers are frozen from the fixed PID before optimization. Divergent/invalid simulations get F=1. Seeds 1001-1020 use `rng(seed,'twister')`; each run uses 25 particles, 30 iterations and 775 evaluations. Positions are normalized to [0,1], velocity is clipped to +/-0.08, and one prescribed initial point is included. ASMC-QLPSO and ASMC-PSO use paired initial populations and random movement factors at a common seed. Ordinary PSO uses [inertia,cognitive,social]=[0.7298,1.49618,1.49618].

Q learning uses a shared 4x4 table, learning rate 0.01, discount 0.9 and exploration probability 0.6. Four distance states use normalized particle-to-global-best distance thresholds [0.75,0.5,0.25]. The action coefficients are [0.95,2.5,0.5], [0.6,1,2], [0.8,2,1.2], [0.5,0.8,2.5]; reward is the old particle fitness minus its new fitness.

| Task | Lower search bounds | Upper search bounds |
| --- | --- | --- |
| 1/2: ASMC | [1,1,0.1,0.1,0.03,0.03] | [50,50,10,10,0.8,0.8] |
| 3: ERLSMC | [0.5,0.5,0.5,0.5,1,1] | [30,30,100,100,100,100] |
| 4: PID | [5,5,0.1,0.1,0.1,0.1] | [400,400,100,100,40,40] |
| 5: ERLSMC-SAT | [0.5,0.5,0.5,0.5,1,1,0.01] | [30,30,100,100,100,100,2] |

Exact prescribed initial points, coefficient updates and tie handling are in `sr_tune_balanced.m`. The packaged optimizer differs from the previously executed revision source only in metric dispatch: a native fallback replaces the mandatory platform-specific MEX dependency.
