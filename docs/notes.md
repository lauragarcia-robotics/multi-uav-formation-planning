# Implementation Notes – Multi-UAV Formation Task

## 1. Formation Reshaping - Path Planning Strategy 

The goal of `getPathsReshapeFormation()` is to compute collision-free paths for multiple UAVs from their initial positions to their desired formation positions.

### Approach

- A 2D grid-based A* planner was used.
- Grid resolution: 0.6 m.
- Obstacles were inflated by a safety margin (~1.2 m) to account for UAV size and safety distance.
- Each UAV is planned sequentially.

### Collision Avoidance Between UAVs

To reduce inter-UAV collisions during reshaping:
- After computing the path of one UAV, its occupied grid cells are marked as *reserved*.
- These reserved cells are treated as obstacles for subsequent UAV planning.

This ensures spatial separation and avoids path overlap.

---

## 2. Target Localization – Multilateration

The method `multilateration()` estimates the 3D position of a signal source based on range measurements from multiple UAVs.

### Mathematical Model

Given:
- UAV positions: p_i
- Measured distances: d_i

We solve:

|| x - p_i || = d_i

This is formulated as a nonlinear least-squares problem.

### Optimization Method

- Gauss–Newton iterative optimization was used.
- Multiple random initial guesses were generated around the formation center.
- The solution with the lowest residual cost was selected.
- The constraint z = 0 was enforced (target lies on the ground plane).

### Robustness Considerations

- Multiple initializations reduce the risk of local minima.
- The best solution is selected based on sum of squared residuals.

---

## 3. Controller Integration

The `update()` method:
- Monitors formation state.
- Calls reshaping when needed.
- Estimates target position using multilateration.
- Uses action handlers to move or reshape formation.

---

## Skills Demonstrated

- Multi-agent path planning
- A* grid search implementation
- Obstacle inflation and safety margins
- Nonlinear optimization (Gauss–Newton)
- Multilateration from range measurements
- Integration within multi-UAV control framework
