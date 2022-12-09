#ifndef BT_SMALLSTEPPCG_CONSTRAINT_SOLVER_H
#define BT_SMALLSTEPPCG_CONSTRAINT_SOLVER_H

#include "btSequentialImpulseConstraintSolver.h"
#include "LinearMath/btThreads.h"

ATTRIBUTE_ALIGNED16(class)
btSmallStepPGSConstraintSolver : public btSequentialImpulseConstraintSolver
{
protected:
	btScalar m_subTimeStep;
	btScalar m_invSubTimeStep;

	virtual btScalar solveGroupCacheFriendlySetup(btCollisionObject * *bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer) BT_OVERRIDE;
	virtual btScalar solveGroupCacheFriendlyIterations(btCollisionObject * *bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer) BT_OVERRIDE;
	virtual btScalar solveGroupCacheFriendlyFinish(btCollisionObject * *bodies, int numBodies, const btContactSolverInfo& infoGlobal) BT_OVERRIDE;

	virtual void convertContacts(btPersistentManifold * *manifoldPtr, int numManifolds, const btContactSolverInfo& infoGlobal) BT_OVERRIDE;
	virtual void convertJoints(btTypedConstraint * *constraints, int numConstraints, const btContactSolverInfo& infoGlobal) BT_OVERRIDE;
	virtual void convertBodies(btCollisionObject * *bodies, int numBodies, const btContactSolverInfo& infoGlobal) BT_OVERRIDE;

	void convertContact(btPersistentManifold * manifold, const btContactSolverInfo& infoGlobal);
	void setupContactConstraint(btSolverConstraint & solverConstraint, int solverBodyIdA, int solverBodyIdB, btManifoldPoint& cp,
								const btContactSolverInfo& infoGlobal, btScalar& relaxation, const btVector3& rel_pos1, const btVector3& rel_pos2);
	void setupFrictionConstraint(btSolverConstraint & solverConstraint, const btVector3& normalAxis, int solverBodyIdA, int solverBodyIdB,
								 btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2,
								 btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation,
								 const btContactSolverInfo& infoGlobal,
								 btScalar desiredVelocity = 0., btScalar cfmSlip = 0.);
	btSolverConstraint& addFrictionConstraint(const btVector3& normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, const btContactSolverInfo& infoGlobal, btScalar desiredVelocity = 0., btScalar cfmSlip = 0.);

	void convertJoint(btSolverConstraint * currentConstraintRow, btTypedConstraint * constraint, const btTypedConstraint::btConstraintInfo1& info1, int solverBodyIdA, int solverBodyIdB, const btContactSolverInfo& infoGlobal);

	void updateConstraints(int iteration, btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer);
	void updateJoint(int iteration, btSolverConstraint* currentConstraintRow, btTypedConstraint* constraint, const btTypedConstraint::btConstraintInfo1& info1, int solverBodyIdA, int solverBodyIdB, const btContactSolverInfo& infoGlobal);
	void updateContact(int iteration, btSolverConstraint& solverConstraint, const btContactSolverInfo& infoGlobal);

	void updateFrictionConstraint(int iteration, btSolverConstraint & solverConstraint,
		int solverBodyIdA, int solverBodyIdB,
		const btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2,
        btScalar relaxation, const btContactSolverInfo& infoGlobal);

	void integrateBodies(int iBegin, int iEnd, btScalar timeStep, const btContactSolverInfo& infoGlobal);

	btScalar solveSingleSplitImpuseIteration(int iteration, btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer);

	void writeBackJoints(int iBegin, int iEnd, const btContactSolverInfo& infoGlobal);
	void updateJointsFeedback(int iBegin, int iEnd, const btContactSolverInfo& infoGlobal);

	void applySplitImpulses(int iBegin, int iEnd, btScalar timeStep, const btContactSolverInfo& infoGlobal);

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btSmallStepPGSConstraintSolver() : btSequentialImpulseConstraintSolver(), m_subTimeStep(0), m_invSubTimeStep(0) {}

	virtual btConstraintSolverType getSolverType() const
	{
		return BT_SMALL_STEP_PGS_SOLVER;
	}
};

#endif