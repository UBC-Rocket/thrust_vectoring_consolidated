# Contributing to TVR

## Branching Strategy

This repo uses a tiered branching model. Each level acts as an integration point for the level below it.

```
main                                          ← protected, release-ready
├── firmware/stage                            ← protected, firmware team integration
│   ├── firmware/flight-controller/stage      ← protected, FC sub-team integration
│   │   ├── firmware/flight-controller/feat/imu-calibration
│   │   └── firmware/flight-controller/bug/spi-timeout
│   ├── firmware/gnss-radio/stage             ← protected
│   │   └── firmware/gnss-radio/feat/gps-cold-start
│   ├── firmware/state-estimation/stage       ← protected
│   │   └── firmware/state-estimation/feat/accel-bias
│   └── firmware/controls/stage               ← protected
│       └── firmware/controls/bug/pid-windup
├── ground-station/stage                      ← protected
│   ├── ground-station/feat/3d-trajectory
│   └── ground-station/bug/serial-disconnect
├── controls/stage                            ← protected
│   └── controls/feat/simulink-model
├── electrical-hardware/stage                 ← protected
│   └── electrical-hardware/feat/power-board-v2
└── mechanical-hardware/stage                 ← protected
    └── mechanical-hardware/feat/fin-redesign
```

### Protected Branches

| Branch | Merges into | Who can merge |
|--------|-------------|---------------|
| `main` | — | Team leads, requires PR approval |
| `*/stage` | `main` | Team leads |
| `*/*/stage` | parent `*/stage` | Sub-team members |

### Rules

1. **`main` is always release-ready.** Only `*/stage` branches can merge into `main` via PR.
2. **`*/stage` branches are integration branches.** They collect work from sub-team stage branches or feature/bug branches, and are the only branches that can PR into `main`.
3. **Sub-team `*/*/stage` branches** (e.g. `firmware/flight-controller/stage`) collect feature and bug branches for that component. They merge up into their parent `*/stage`.
4. **Feature and bug branches** are where day-to-day work happens. They are not protected and can be freely created and deleted.

### Branch Naming

Work branches follow the pattern:

```
<team>/<component>/<type>/<short-description>
```

| Type | Use for |
|------|---------|
| `feat/` | New features or enhancements |
| `bug/` | Bug fixes |
| `refactor/` | Code restructuring without behavior change |
| `test/` | Adding or updating tests |

Examples:
- `firmware/flight-controller/feat/sd-log-compression`
- `firmware/gnss-radio/bug/uart-overrun`
- `ground-station/feat/map-panel`
- `electrical-hardware/feat/imu-breakout-v3`

For top-level team branches without a sub-component, omit the component:
- `controls/feat/thrust-curve-model`
- `mechanical-hardware/feat/nose-cone-v2`

## Pull Request Workflow

1. Create a feature/bug branch off the appropriate `stage` branch
2. Do your work, commit with clear messages
3. Push and open a PR targeting the `stage` branch you branched from
4. Get review from at least one team member
5. Squash-merge or merge into `stage`
6. When `stage` is tested and ready, a team lead opens a PR from `stage` → parent `stage` (or `main`)

### PR Requirements

- All CI checks must pass
- At least one approving review
- Branch must be up to date with the target branch
- No unresolved review comments

## Commit Messages

Use short, descriptive commit messages:

```
<type>: <short summary>

<optional body>
```

Types: `feat`, `fix`, `refactor`, `test`, `docs`, `ci`, `chore`

Examples:
- `feat: add barometer altitude fusion to EKF`
- `fix: prevent SPI queue overflow on high-rate IMU`
- `docs: update SPI DMA overview with timing diagram`
- `ci: add state_estimation tests to firmware workflow`

## Development Setup

See the top-level [README.md](README.md) for build prerequisites and instructions.

### Running Tests Before Pushing

```bash
# State estimation
cd firmware/libs/state_estimation/tests
cmake -B build && cmake --build build && ctest --test-dir build

# Controls
cd firmware/libs/controls/tests
cmake -B build && cmake --build build && ctest --test-dir build
```

### Code Style

- C code: follow existing style in the file you're editing
- Snake case for file names, function names, and variables
- Prefix library headers with their directory (e.g. `#include "state_estimation/ekf.h"`)
