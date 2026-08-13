# Small Step PGS Solver for Bullet Physics
This repository extends Bullet Physics with a small step Projected Gauss-Seidel (PGS) constraint solver for rigid-body simulation.

The solver divides each timestep into multiple smaller substeps and performs one PGS iteration per substep. This improves constraint convergence, particularly for simulations with large mass ratios.

## Key Modifications
* `btSmallStepPGSConstraintSolver`: A new solver class extending btSequentialImpulseConstraintSolver that subdivides the simulation timestep and interleaves constraint solving with body integration.

* Substepping loop: Each substep updates Jacobians and effective masses based on current body states, executes one PGS iteration, then integrates velocities and transforms before the next substep.

* Baumgarte stabilization tuning: Adds `m_maxContactConstraintStablizationSpeed` to cap stabilization impulses and mitigate energy injection instabilities inherent to small-stepping.

* DiscreteDynamicsWorld integration: Adjusts motion prediction and transform handling to support the solver's substepped integration model.

## Results and references
See the technical report: [*Solving rigid body constraints with a small step solver*](docs/technical-report.pdf).