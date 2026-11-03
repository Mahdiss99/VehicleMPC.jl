# NMPC runtime benchmark

- Julia 1.11.5, NT x86_64, CPU: 13th Gen Intel(R) Core(TM) i9-13900HX (32 logical cores), Julia threads: 1
- JuMP 1.32.1, Ipopt.jl 1.16.0, Ipopt 3.14.19 (default linear solver, MUMPS)
- 250 consecutive closed-loop control periods per scenario and round, 3 rounds with alternating order (750 solves per scenario); dt = 0.1 s; fresh controller per run
- Times are wall-clock milliseconds of one `solve!` call (parameter update + initial guess + Ipopt + validation). The plant simulation is excluded.

## Start-up (includes compilation)

| quantity | value |
|:--|--:|
| first model construction (RK4, N = 20) | 4241 ms |
| first solve, cold (includes compilation) | 8558 ms |
| second solve on the same controller | 274.3 ms |

## End-to-end `solve!` latency

All times in milliseconds. "Round medians" is the range of the per-round medians, which shows run-to-run variability.

| track | integrator | N | init | solves | mean | median | round medians | p95 | p99 | max | > 80 ms | > 100 ms | Ipopt iterations: mean (median / max) | rejected |
|:--|:--|--:|:--|--:|--:|--:|:-:|--:|--:|--:|--:|--:|--:|--:|
| circle | RK4 | 20 | warm | 750 | 38.5 | 38.4 | 38.2–39.0 | 48.9 | 56.7 | 66.9 | 0 (0.0 %) | 0 (0.0 %) | 18.1 (18 / 20) | 0 |
| circle | RK4 | 20 | cold | 750 | 31.3 | 30.4 | 28.4–32.9 | 42.2 | 52.8 | 93.7 | 1 (0.1 %) | 0 (0.0 %) | 15.1 (15 / 17) | 0 |
| circle | Euler | 20 | warm | 750 | 20.6 | 20.6 | 17.9–21.8 | 27.5 | 31.3 | 40.7 | 0 (0.0 %) | 0 (0.0 %) | 19.3 (19 / 21) | 0 |
| circle | Euler | 20 | cold | 750 | 17.4 | 17.1 | 14.8–18.3 | 24.3 | 30.7 | 39.5 | 0 (0.0 %) | 0 (0.0 %) | 16.3 (16 / 18) | 0 |
| circle | RK4 | 10 | warm | 750 | 18.6 | 18.4 | 15.7–19.9 | 25.8 | 29.4 | 39.3 | 0 (0.0 %) | 0 (0.0 %) | 15.2 (15 / 18) | 0 |
| circle | RK4 | 30 | warm | 750 | 50.6 | 49.7 | 47.8–53.4 | 69.4 | 78.8 | 88.3 | 8 (1.1 %) | 0 (0.0 %) | 18.4 (18 / 20) | 0 |
| straight | RK4 | 20 | warm | 750 | 16.7 | 16.3 | 13.7–19.1 | 25.3 | 30.6 | 52.9 | 0 (0.0 %) | 0 (0.0 %) | 8.2 (8 / 16) | 0 |
| straight | RK4 | 20 | cold | 750 | 17.5 | 16.8 | 14.6–19.4 | 27.0 | 34.9 | 69.8 | 0 (0.0 %) | 0 (0.0 %) | 8.2 (8 / 16) | 0 |

## Mean time per component [ms]

| track | integrator | N | init | model build | parameter update | initial guess | `optimize!` (wall) | Ipopt-reported solve | Ipopt per iteration | extraction + validation |
|:--|:--|--:|:--|--:|--:|--:|--:|--:|--:|--:|
| circle | RK4 | 20 | warm | 13.1 | 0.007 | 0.191 | 38.20 | 38.17 | 2.11 | 0.134 |
| circle | RK4 | 20 | cold | 16.7 | 0.006 | 0.179 | 30.93 | 30.83 | 2.04 | 0.131 |
| circle | Euler | 20 | warm | 6.7 | 0.005 | 0.098 | 20.32 | 20.31 | 1.05 | 0.130 |
| circle | Euler | 20 | cold | 6.6 | 0.005 | 0.091 | 17.17 | 17.15 | 1.05 | 0.121 |
| circle | RK4 | 10 | warm | 7.4 | 0.005 | 0.095 | 18.38 | 18.33 | 1.21 | 0.075 |
| circle | RK4 | 30 | warm | 17.7 | 0.008 | 0.275 | 50.11 | 50.06 | 2.72 | 0.190 |
| straight | RK4 | 20 | warm | 9.5 | 0.005 | 0.150 | 16.38 | 16.25 | 1.99 | 0.117 |
| straight | RK4 | 20 | cold | 10.7 | 0.005 | 0.155 | 17.17 | 17.04 | 2.08 | 0.120 |

## Validation

| track | integrator | N | init | closed-loop status | max dynamics residual | max knot constraint violation | plant operating violations |
|:--|:--|--:|:--|:--|--:|--:|--:|
| circle | RK4 | 20 | warm | success | 5.8e-11 | 0.0e+00 | 0 |
| circle | RK4 | 20 | cold | success | 5.8e-11 | 0.0e+00 | 0 |
| circle | Euler | 20 | warm | success | 7.8e-11 | 0.0e+00 | 0 |
| circle | Euler | 20 | cold | success | 7.8e-11 | 0.0e+00 | 0 |
| circle | RK4 | 10 | warm | success | 9.6e-10 | 0.0e+00 | 0 |
| circle | RK4 | 30 | warm | success | 6.2e-11 | 0.0e+00 | 0 |
| straight | RK4 | 20 | warm | success | 4.6e-08 | 0.0e+00 | 0 |
| straight | RK4 | 20 | cold | success | 4.6e-08 | 0.0e+00 | 0 |
