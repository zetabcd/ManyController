# Nonlinear MPC third-party dependencies

This directory contains the solver dependencies used by the four optional
`px4ctrl` nonlinear MPC backends.  The `px4ctrl` CMake file deliberately uses
`NO_DEFAULT_PATH` for these packages so that a host installation cannot be
selected accidentally.

| Directory | Used by |
| --- | --- |
| `optimization/` | Ipopt, MUMPS and NLopt (`ipopt+eigen`, `nlopt+eigen`) |
| `casadi/` | CasADi and its Ipopt plugin (`ipopt+casadi`) |
| `acados/` | acados, HPIPM, BLASFEO and the generated `px4ctrl_nmpc` solver |

The Debian NLopt package also ships documentation examples.  Their upstream
`CMakeLists.txt` is intentionally omitted here because it references source
tree-only files such as `src/util/timer.c` that are not part of the binary
development package.  Those examples are not required by `px4ctrl`.

The acados generated code can be regenerated after changing its model with:

```bash
ACADOS_SOURCE_DIR=/home/sun/ipopt_test/acados \
PYTHONPATH=/home/sun/ipopt_test/acados/interfaces/acados_template \
python3 script/generate_px4ctrl_acados_nmpc.py
```

The public controller selector is `PX4CTRL_PRIMARY_CONTROLLER` in
`include/px4ctrl/px4ctrlfsm.h`:

- `0`: original `QuadControl` (kept intact)
- `1`: `GptMpcControl` (default)
- `2`: Ipopt + Eigen
- `3`: acados
- `4`: NLopt + Eigen
- `5`: Ipopt + CasADi

All four nonlinear backends consume the same `GptTrajectoryResult` through
`setTrajectory()`.  The common adapter extracts a horizon-sized moving window
from the full trajectory, converts thrust from acceleration to force at the
controller boundary, and retains the last valid command when a solver fails.
