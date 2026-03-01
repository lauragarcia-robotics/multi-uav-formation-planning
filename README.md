# Multi-UAV Formation Reshaping + Multilateration (CVUT MRS)

Implementation of formation reshaping for multiple UAVs and target localization from range measurements.

## What I implemented

### Formation reshaping (path planning)
- A* grid planning for each UAV from initial to final formation positions
- Safety distance handled by obstacle inflation
- Sequential planning with reserved grid cells to reduce inter-UAV collisions

### Target localization (multilateration)
- Nonlinear least-squares optimization using Gauss–Newton
- Multiple random initializations and best-cost selection
- Constraint: target lies on the ground plane (z = 0)

## Tech stack
- C++ (Eigen)
- Course simulation environment (CVUT MRS)

## Repository content
This repo contains only my implementation files (course framework excluded):
- `src/formation.cpp`
- (optional) `include/...`
- `config/...`
