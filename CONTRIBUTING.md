# Contributing to TVR

## Branching Strategy

This repo uses a simple two-tier branching model. Each team has a `stage` branch that collects work before it merges into `main`.

```
main                                          ← protected, release-ready
├── embedded-software/stage                   ← protected, team integration
│   ├── embedded-software/feat/sd-log-compression
│   └── embedded-software/bug/spi-timeout
├── controls/stage                            ← protected
│   └── controls/feat/thrust-curve-model
├── electrical-hardware/stage                 ← protected
│   └── electrical-hardware/feat/power-board-v2
└── mechanical-hardware/stage                 ← protected
    └── mechanical-hardware/feat/fin-redesign
```

### Protected Branches

| Branch | Merges into | Who can approve |
|--------|-------------|-----------------|
| `main` | — | Own team lead (via CODEOWNERS) |
| `*/stage` | `main` | Any team member |

### Team Leads & Required Reviewers

PRs from `*/stage` into `main` require approval from the team's lead(s). This is enforced by GitHub Rulesets + CODEOWNERS.

| Team | Stage Branch | Lead(s) |
|---|---|---|
| Embedded Software | `embedded-software/stage` | @benedikthoward, @Shengw3n |
| Controls | `controls/stage` | @p14n0f0rt3 |
| Electrical Hardware | `electrical-hardware/stage` | @ShadabMizan |
| Mechanical Hardware | `mechanical-hardware/stage` | @Joshxck |

> When leads change, update this table and the corresponding GitHub Ruleset (Settings → Rules → Rulesets).

### Rules

1. **`main` is always release-ready.** Only `*/stage` branches merge into `main` via PR, with lead approval.
2. **`*/stage` branches are team integration branches.** Feature and bug branches PR into their team's `stage` branch. Any team member can approve.
3. **Feature and bug branches** are where day-to-day work happens. They are not protected and can be freely created and deleted.

### Branch Naming

Work branches follow the pattern:

```
<team>/<type>/<short-description>
```

| Type | Use for |
|------|---------|
| `feat/` | New features or enhancements |
| `bug/` | Bug fixes |
| `refactor/` | Code restructuring without behavior change |
| `test/` | Adding or updating tests |

Examples:
- `embedded-software/feat/imu-calibration`
- `embedded-software/bug/uart-overrun`
- `controls/feat/simulink-model`
- `electrical-hardware/feat/imu-breakout-v3`
- `mechanical-hardware/feat/nose-cone-v2`

## Pull Request Workflow

1. Create a feature/bug branch off your team's `stage` branch
2. Do your work, commit with clear messages
3. Push and open a PR targeting your team's `stage` branch
4. Get review from at least one team member
5. Squash-merge or merge into `stage`
6. When `stage` is tested and ready, a team lead opens a PR from `stage` → `main`

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
cd embedded-software/firmware/libs/state_estimation/tests
cmake -B build && cmake --build build && ctest --test-dir build

# Controls
cd embedded-software/firmware/libs/controls/tests
cmake -B build && cmake --build build && ctest --test-dir build
```

### Code Style

- C code: follow existing style in the file you're editing
- Snake case for file names, function names, and variables
- Prefix library headers with their directory (e.g. `#include "state_estimation/ekf.h"`)
