# Small Step PGS Solver for Bullet Physics
This repository extends Bullet Physics with a small step Projected Gauss-Seidel (PGS) constraint solver for rigid body simulation.

The solver divides each timestep into multiple smaller substeps and performs one PGS iteration per substep. This improves constraint convergence, particularly for simulations with large mass ratios.

## Key modifications
* `btSmallStepPGSConstraintSolver`: A new solver class extending btSequentialImpulseConstraintSolver that subdivides the simulation timestep and interleaves constraint solving with body integration.

* Substepping loop: Each substep updates Jacobians and effective masses based on current body states, executes one PGS iteration, then integrates velocities and transforms before the next substep.

* Baumgarte stabilization tuning: Adds `m_maxContactConstraintStablizationSpeed` to cap stabilization impulses and mitigate energy injection instabilities inherent to small-stepping.

* Pre-stabilization strategy: Applies position-based constraint stabilization before solver execution to reduce deep penetrations and limit subsequent energy injection by the Baumgarte stablizer.

* `btDiscreteDynamicsWorld` integration: Adjusts motion prediction and transform handling to support the solver's substepped integration model.

## Results and references
The solver is validated on scenes with 200:1 mass ratios (suspended chain, heavy box stack), where standard PGS fails to converge even at high iteration counts.

See the technical report for more detials: [*Solving rigid body constraints with a small step solver*](docs/technical-report.pdf).