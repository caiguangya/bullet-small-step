#include "btSmallStepPGSConstraintSolver.h"
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include <string.h>

btScalar btSmallStepPGSConstraintSolver::solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	m_subTimeStep = infoGlobal.m_timeStep / infoGlobal.m_numIterations;
	m_invSubTimeStep = btScalar(1) / m_subTimeStep;
	return btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
}

btScalar btSmallStepPGSConstraintSolver::solveGroupCacheFriendlyFinish(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& infoGlobal) {
	BT_PROFILE("solveGroupCacheFriendlyFinish");

	if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
	{
		writeBackContacts(0, m_tmpSolverContactConstraintPool.size(), infoGlobal);
	}

	writeBackJoints(0, m_tmpSolverNonContactConstraintPool.size(), infoGlobal);

	writeBackBodies(0, m_tmpSolverBodyPool.size(), infoGlobal);

	m_tmpSolverContactConstraintPool.resizeNoInitialize(0);
	m_tmpSolverNonContactConstraintPool.resizeNoInitialize(0);
	m_tmpSolverContactFrictionConstraintPool.resizeNoInitialize(0);
	m_tmpSolverContactRollingFrictionConstraintPool.resizeNoInitialize(0);

	m_tmpNonContactConstraintsPreAppliedImpulse.resizeNoInitialize(0);

	m_tmpSolverBodyPool.resizeNoInitialize(0);

	return btScalar(0);
}

btScalar btSmallStepPGSConstraintSolver::solveGroupCacheFriendlyIterations(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer) {
	BT_PROFILE("solveGroupCacheFriendlyIterations");
	{
		solveGroupCacheFriendlySplitImpulseIterations(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
		applySplitImpulses(0, m_tmpSolverBodyPool.size(), infoGlobal.m_timeStep, infoGlobal);

		int maxIterations = infoGlobal.m_numIterations;
		for (int iteration = 0; iteration < maxIterations; iteration++)
		{
			applyExternalImpulses(0, m_tmpSolverBodyPool.size(), infoGlobal);

			updateConstraints(iteration, bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
			solveSingleIteration(iteration, bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
			integrateBodies(0, m_tmpSolverBodyPool.size(), m_subTimeStep, infoGlobal);

			updateJointsFeedback(0, m_tmpSolverNonContactConstraintPool.size(), infoGlobal);
		}
	}

	return btScalar(0);
}


void btSmallStepPGSConstraintSolver::convertContacts(btPersistentManifold** manifoldPtr, int numManifolds, const btContactSolverInfo& infoGlobal)
{
	for (int i = 0; i < numManifolds; i++)
	{
		convertContact(manifoldPtr[i], infoGlobal);
	}
}

void btSmallStepPGSConstraintSolver::convertContact(btPersistentManifold* manifold, const btContactSolverInfo& infoGlobal) 
{
	btCollisionObject *colObj0 = 0, *colObj1 = 0;

	colObj0 = (btCollisionObject*)manifold->getBody0();
	colObj1 = (btCollisionObject*)manifold->getBody1();

	int solverBodyIdA = getOrInitSolverBody(*colObj0, m_subTimeStep);
	int solverBodyIdB = getOrInitSolverBody(*colObj1, m_subTimeStep);

	btSolverBody* solverBodyA = &m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody* solverBodyB = &m_tmpSolverBodyPool[solverBodyIdB];

	///avoid collision response between two static objects
	if (!solverBodyA || (solverBodyA->m_invMass.fuzzyZero() && (!solverBodyB || solverBodyB->m_invMass.fuzzyZero())))
		return;

	for (int j = 0; j < manifold->getNumContacts(); j++)
	{
		btManifoldPoint& cp = manifold->getContactPoint(j);

		if (cp.getDistance() <= manifold->getContactProcessingThreshold())
		{
			btVector3 rel_pos1;
			btVector3 rel_pos2;
			btScalar relaxation;

			int frictionIndex = m_tmpSolverContactConstraintPool.size();
			btSolverConstraint& solverConstraint = m_tmpSolverContactConstraintPool.expandNonInitializing();

			solverConstraint.m_solverBodyIdA = solverBodyIdA;
			solverConstraint.m_solverBodyIdB = solverBodyIdB;

			solverConstraint.m_originalContactPoint = &cp;

			const btVector3& pos1 = cp.getPositionWorldOnA();
			const btVector3& pos2 = cp.getPositionWorldOnB();

			rel_pos1 = pos1 - colObj0->getWorldTransform().getOrigin();
			rel_pos2 = pos2 - colObj1->getWorldTransform().getOrigin();

			btVector3 vel1;
			btVector3 vel2;

			solverBodyA->getVelocityInLocalPointNoDelta(rel_pos1, vel1);
			solverBodyB->getVelocityInLocalPointNoDelta(rel_pos2, vel2);

			btVector3 vel = vel1 - vel2;
			btScalar rel_vel = cp.m_normalWorldOnB.dot(vel);

			setupContactConstraint(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal, relaxation, rel_pos1, rel_pos2);

			/////setup the friction constraints

			solverConstraint.m_frictionIndex = m_tmpSolverContactFrictionConstraintPool.size();

			///Bullet has several options to set the friction directions
			///By default, each contact has only a single friction direction that is recomputed automatically very frame
			///based on the relative linear velocity.
			///If the relative velocity it zero, it will automatically compute a friction direction.

			///You can also enable two friction directions, using the SOLVER_USE_2_FRICTION_DIRECTIONS.
			///In that case, the second friction direction will be orthogonal to both contact normal and first friction direction.
			///
			///If you choose SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION, then the friction will be independent from the relative projected velocity.
			///
			///The user can manually override the friction directions for certain contacts using a contact callback,
			///and use contactPoint.m_contactPointFlags |= BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED
			///In that case, you can set the target relative motion in each friction direction (cp.m_contactMotion1 and cp.m_contactMotion2)
			///this will give a conveyor belt effect
			///

			if (!(infoGlobal.m_solverMode & SOLVER_ENABLE_FRICTION_DIRECTION_CACHING) || !(cp.m_contactPointFlags & BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED))
			{
				cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
				btScalar lat_rel_vel = cp.m_lateralFrictionDir1.length2();
				if (!(infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION) && lat_rel_vel > SIMD_EPSILON)
				{
					cp.m_lateralFrictionDir1 *= 1.f / btSqrt(lat_rel_vel);
					applyAnisotropicFriction(colObj0, cp.m_lateralFrictionDir1, btCollisionObject::CF_ANISOTROPIC_FRICTION);
					applyAnisotropicFriction(colObj1, cp.m_lateralFrictionDir1, btCollisionObject::CF_ANISOTROPIC_FRICTION);
					addFrictionConstraint(cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal);

					if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
					{
						cp.m_lateralFrictionDir2 = cp.m_lateralFrictionDir1.cross(cp.m_normalWorldOnB);
						cp.m_lateralFrictionDir2.normalize();  //??
						applyAnisotropicFriction(colObj0, cp.m_lateralFrictionDir2, btCollisionObject::CF_ANISOTROPIC_FRICTION);
						applyAnisotropicFriction(colObj1, cp.m_lateralFrictionDir2, btCollisionObject::CF_ANISOTROPIC_FRICTION);
						addFrictionConstraint(cp.m_lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal);
					}
				}
				else
				{
					btPlaneSpace1(cp.m_normalWorldOnB, cp.m_lateralFrictionDir1, cp.m_lateralFrictionDir2);

					applyAnisotropicFriction(colObj0, cp.m_lateralFrictionDir1, btCollisionObject::CF_ANISOTROPIC_FRICTION);
					applyAnisotropicFriction(colObj1, cp.m_lateralFrictionDir1, btCollisionObject::CF_ANISOTROPIC_FRICTION);
					addFrictionConstraint(cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal);

					if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
					{
						applyAnisotropicFriction(colObj0, cp.m_lateralFrictionDir2, btCollisionObject::CF_ANISOTROPIC_FRICTION);
						applyAnisotropicFriction(colObj1, cp.m_lateralFrictionDir2, btCollisionObject::CF_ANISOTROPIC_FRICTION);
						addFrictionConstraint(cp.m_lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal);
					}

					if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) && (infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION))
					{
						cp.m_contactPointFlags |= BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED;
					}
				}
			}
			else
			{
				addFrictionConstraint(cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal, cp.m_contactMotion1, cp.m_frictionCFM);

				if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
				{
					addFrictionConstraint(cp.m_lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal, cp.m_contactMotion2, cp.m_frictionCFM);
				}
			}
			setFrictionConstraintImpulse(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal);
		}
	}
}

void btSmallStepPGSConstraintSolver::setupFrictionConstraint(btSolverConstraint& solverConstraint, const btVector3& normalAxis, int solverBodyIdA, int solverBodyIdB, btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, const btContactSolverInfo& infoGlobal, btScalar desiredVelocity, btScalar cfmSlip)
{
	btSolverBody& solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody& solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];

	btRigidBody* bodyA = m_tmpSolverBodyPool[solverBodyIdA].m_originalBody;
	btRigidBody* bodyB = m_tmpSolverBodyPool[solverBodyIdB].m_originalBody;

	solverConstraint.m_solverBodyIdA = solverBodyIdA;
	solverConstraint.m_solverBodyIdB = solverBodyIdB;

	solverConstraint.m_friction = cp.m_combinedFriction;
	solverConstraint.m_originalContactPoint = 0;

	solverConstraint.m_appliedImpulse = 0.f;
	solverConstraint.m_appliedPushImpulse = 0.f;

	if (bodyA)
	{
		solverConstraint.m_contactNormal1 = normalAxis;
	}
	else
	{
		solverConstraint.m_contactNormal1.setZero();
	}

	if (bodyB)
	{
		solverConstraint.m_contactNormal2 = -normalAxis;
	}
	else
	{
		solverConstraint.m_contactNormal2.setZero();
	}

	solverConstraint.m_relpos1CrossNormal.setZero();
	solverConstraint.m_angularComponentA.setZero();
	solverConstraint.m_relpos2CrossNormal.setZero();
	solverConstraint.m_angularComponentB.setZero();

	solverConstraint.m_rhs = 0.f;
	solverConstraint.m_rhsPenetration = 0.f;
	solverConstraint.m_cfm = cfmSlip;
	solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
	solverConstraint.m_upperLimit = solverConstraint.m_friction;
}

btSolverConstraint& btSmallStepPGSConstraintSolver::addFrictionConstraint(const btVector3& normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, const btContactSolverInfo& infoGlobal, btScalar desiredVelocity, btScalar cfmSlip)
{
	btSolverConstraint& solverConstraint = m_tmpSolverContactFrictionConstraintPool.expandNonInitializing();
	solverConstraint.m_frictionIndex = frictionIndex;
	solverConstraint.m_targetVel = desiredVelocity;
	setupFrictionConstraint(solverConstraint, normalAxis, solverBodyIdA, solverBodyIdB, cp, rel_pos1, rel_pos2,
							colObj0, colObj1, relaxation, infoGlobal, desiredVelocity, cfmSlip);
	return solverConstraint;
}

void btSmallStepPGSConstraintSolver::setupContactConstraint(btSolverConstraint& solverConstraint, int solverBodyIdA, int solverBodyIdB, btManifoldPoint& cp,
	const btContactSolverInfo& infoGlobal, btScalar& relaxation, const btVector3& rel_pos1, const btVector3& rel_pos2) 
{
	btSolverBody* bodyA = &m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody* bodyB = &m_tmpSolverBodyPool[solverBodyIdB];

	btRigidBody* rb0 = bodyA->m_originalBody;
	btRigidBody* rb1 = bodyB->m_originalBody;

	relaxation = infoGlobal.m_sor;
	btScalar invTimeStep = m_invSubTimeStep;

	//cfm = 1 /       ( dt * kp + kd )
	//erp = dt * kp / ( dt * kp + kd )

	btScalar cfm = infoGlobal.m_globalCfm;
	btScalar erp = infoGlobal.m_erp2;

	if ((cp.m_contactPointFlags & BT_CONTACT_FLAG_HAS_CONTACT_CFM) || (cp.m_contactPointFlags & BT_CONTACT_FLAG_HAS_CONTACT_ERP))
	{
		if (cp.m_contactPointFlags & BT_CONTACT_FLAG_HAS_CONTACT_CFM)
			cfm = cp.m_contactCFM;
		if (cp.m_contactPointFlags & BT_CONTACT_FLAG_HAS_CONTACT_ERP)
			erp = cp.m_contactERP;
	}
	else
	{
		if (cp.m_contactPointFlags & BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING)
		{
			btScalar denom = (m_subTimeStep * cp.m_combinedContactStiffness1 + cp.m_combinedContactDamping1);
			if (denom < SIMD_EPSILON)
			{
				denom = SIMD_EPSILON;
			}
			cfm = btScalar(1) / denom;
			erp = (m_subTimeStep * cp.m_combinedContactStiffness1) / denom;
		}
	}

	cfm *= invTimeStep;

	btVector3 torqueAxis0 = rel_pos1.cross(cp.m_normalWorldOnB);
	solverConstraint.m_angularComponentA = rb0 ? rb0->getInvInertiaTensorWorld() * torqueAxis0 * rb0->getAngularFactor() : btVector3(0, 0, 0);
	btVector3 torqueAxis1 = rel_pos2.cross(cp.m_normalWorldOnB);
	solverConstraint.m_angularComponentB = rb1 ? rb1->getInvInertiaTensorWorld() * -torqueAxis1 * rb1->getAngularFactor() : btVector3(0, 0, 0);

	{
#ifdef COMPUTE_IMPULSE_DENOM
		btScalar denom0 = rb0->computeImpulseDenominator(pos1, cp.m_normalWorldOnB);
		btScalar denom1 = rb1->computeImpulseDenominator(pos2, cp.m_normalWorldOnB);
#else
		btVector3 vec;
		btScalar denom0 = 0.f;
		btScalar denom1 = 0.f;
		if (rb0)
		{
			vec = (solverConstraint.m_angularComponentA).cross(rel_pos1);
			denom0 = rb0->getInvMass() + cp.m_normalWorldOnB.dot(vec);
		}
		if (rb1)
		{
			vec = (-solverConstraint.m_angularComponentB).cross(rel_pos2);
			denom1 = rb1->getInvMass() + cp.m_normalWorldOnB.dot(vec);
		}
#endif  //COMPUTE_IMPULSE_DENOM

		btScalar denom = relaxation / (denom0 + denom1 + cfm);
		solverConstraint.m_jacDiagABInv = denom;
	}

	if (rb0)
	{
		solverConstraint.m_contactNormal1 = cp.m_normalWorldOnB;
		solverConstraint.m_relpos1CrossNormal = torqueAxis0;
	}
	else
	{
		solverConstraint.m_contactNormal1.setZero();
		solverConstraint.m_relpos1CrossNormal.setZero();
	}
	if (rb1)
	{
		solverConstraint.m_contactNormal2 = -cp.m_normalWorldOnB;
		solverConstraint.m_relpos2CrossNormal = -torqueAxis1;
	}
	else
	{
		solverConstraint.m_contactNormal2.setZero();
		solverConstraint.m_relpos2CrossNormal.setZero();
	}

	btScalar restitution = 0.f;
	btScalar penetration = cp.getDistance() + infoGlobal.m_linearSlop;

	{
		btVector3 vel1, vel2;

		vel1 = rb0 ? rb0->getVelocityInLocalPoint(rel_pos1) : btVector3(0, 0, 0);
		vel2 = rb1 ? rb1->getVelocityInLocalPoint(rel_pos2) : btVector3(0, 0, 0);

		btVector3 vel = vel1 - vel2;
		btScalar rel_vel = cp.m_normalWorldOnB.dot(vel);

		solverConstraint.m_friction = cp.m_combinedFriction;

		restitution = restitutionCurve(rel_vel, cp.m_combinedRestitution, infoGlobal.m_restitutionVelocityThreshold);
		if (restitution <= btScalar(0.))
		{
			restitution = 0.f;
		};
	}

	solverConstraint.m_appliedImpulse = 0.f;
	solverConstraint.m_appliedPushImpulse = 0.f;

	{
		btScalar positionalError = 0;
		btScalar stablizeVelocity = 0;
		if (penetration > 0)
		{
			stablizeVelocity = penetration / infoGlobal.m_timeStep;
		}
		else
		{
			positionalError = -penetration * erp / infoGlobal.m_timeStep;
		}

		solverConstraint.m_rhs = 0;
		if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
		{
			solverConstraint.m_rhsPenetration = 0.f;
		}
		else
		{
			solverConstraint.m_rhsPenetration = positionalError * solverConstraint.m_jacDiagABInv;
		}

		solverConstraint.m_targetVel = restitution - stablizeVelocity;
		solverConstraint.m_cfm = cfm * solverConstraint.m_jacDiagABInv;
		solverConstraint.m_erp = erp;
		solverConstraint.m_lowerLimit = 0;
		solverConstraint.m_upperLimit = 1e10f;
	}
}


void btSmallStepPGSConstraintSolver::convertJoints(btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal) 
{
	BT_PROFILE("convertJoints");
	for (int j = 0; j < numConstraints; j++)
	{
		btTypedConstraint* constraint = constraints[j];
		constraint->buildJacobian();
		constraint->internalSetAppliedImpulse(0.0f);
	}

	int totalNumRows = 0;

	m_tmpConstraintSizesPool.resizeNoInitialize(numConstraints);
	//calculate the total number of contraint rows
	for (int i = 0; i < numConstraints; i++)
	{
		btTypedConstraint::btConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];
		btJointFeedback* fb = constraints[i]->getJointFeedback();
		if (fb)
		{
			fb->m_appliedForceBodyA.setZero();
			fb->m_appliedTorqueBodyA.setZero();
			fb->m_appliedForceBodyB.setZero();
			fb->m_appliedTorqueBodyB.setZero();
		}

		if (constraints[i]->isEnabled())
		{
			constraints[i]->getInfo1(&info1);
		}
		else
		{
			info1.m_numConstraintRows = 0;
			info1.nub = 0;
		}
		totalNumRows += info1.m_numConstraintRows;
	}
	m_tmpSolverNonContactConstraintPool.resizeNoInitialize(totalNumRows);
	m_tmpNonContactConstraintsPreAppliedImpulse.resize(totalNumRows, 0);

	///setup the btSolverConstraints
	int currentRow = 0;

	for (int i = 0; i < numConstraints; i++)
	{
		const btTypedConstraint::btConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];

		if (info1.m_numConstraintRows)
		{
			btAssert(currentRow < totalNumRows);

			btSolverConstraint* currentConstraintRow = &m_tmpSolverNonContactConstraintPool[currentRow];
			btTypedConstraint* constraint = constraints[i];
			btRigidBody& rbA = constraint->getRigidBodyA();
			btRigidBody& rbB = constraint->getRigidBodyB();

			int solverBodyIdA = getOrInitSolverBody(rbA, m_subTimeStep);
			int solverBodyIdB = getOrInitSolverBody(rbB, m_subTimeStep);

			convertJoint(currentConstraintRow, constraint, info1, solverBodyIdA, solverBodyIdB, infoGlobal);
		}
		currentRow += info1.m_numConstraintRows;
	}
}

void btSmallStepPGSConstraintSolver::convertJoint(btSolverConstraint* currentConstraintRow, btTypedConstraint* constraint, const btTypedConstraint::btConstraintInfo1& info1, int solverBodyIdA, int solverBodyIdB, const btContactSolverInfo& infoGlobal)
{
	const btRigidBody& rbA = constraint->getRigidBodyA();
	const btRigidBody& rbB = constraint->getRigidBodyB();

	const btSolverBody* bodyAPtr = &m_tmpSolverBodyPool[solverBodyIdA];
	const btSolverBody* bodyBPtr = &m_tmpSolverBodyPool[solverBodyIdB];

	for (int j = 0; j < info1.m_numConstraintRows; j++)
	{
		memset(&currentConstraintRow[j], 0, sizeof(btSolverConstraint));
		currentConstraintRow[j].m_lowerLimit = -SIMD_INFINITY;
		currentConstraintRow[j].m_upperLimit = SIMD_INFINITY;
		currentConstraintRow[j].m_appliedImpulse = 0.f;
		currentConstraintRow[j].m_appliedPushImpulse = 0.f;
		currentConstraintRow[j].m_solverBodyIdA = solverBodyIdA;
		currentConstraintRow[j].m_solverBodyIdB = solverBodyIdB;
		currentConstraintRow[j].m_overrideNumSolverIterations = infoGlobal.m_numIterations;
	}

	// these vectors are already cleared in initSolverBody, no need to redundantly clear again
	btAssert(bodyAPtr->getDeltaLinearVelocity().isZero());
	btAssert(bodyAPtr->getDeltaAngularVelocity().isZero());
	btAssert(bodyAPtr->getPushVelocity().isZero());
	btAssert(bodyAPtr->getTurnVelocity().isZero());
	btAssert(bodyBPtr->getDeltaLinearVelocity().isZero());
	btAssert(bodyBPtr->getDeltaAngularVelocity().isZero());
	btAssert(bodyBPtr->getPushVelocity().isZero());
	btAssert(bodyBPtr->getTurnVelocity().isZero());

	btTypedConstraint::btConstraintInfo2 info2;
	info2.fps = m_invSubTimeStep;
	info2.erp = infoGlobal.m_erp;
	info2.m_J1linearAxis = currentConstraintRow->m_contactNormal1;
	info2.m_J1angularAxis = currentConstraintRow->m_relpos1CrossNormal;
	info2.m_J2linearAxis = currentConstraintRow->m_contactNormal2;
	info2.m_J2angularAxis = currentConstraintRow->m_relpos2CrossNormal;
	info2.rowskip = sizeof(btSolverConstraint) / sizeof(btScalar);  //check this
																	///the size of btSolverConstraint needs be a multiple of btScalar
	btAssert(info2.rowskip * sizeof(btScalar) == sizeof(btSolverConstraint));
	info2.m_constraintError = &currentConstraintRow->m_rhs;
	currentConstraintRow->m_cfm = infoGlobal.m_globalCfm;
	info2.m_damping = infoGlobal.m_damping;
	info2.cfm = &currentConstraintRow->m_cfm;
	info2.m_lowerLimit = &currentConstraintRow->m_lowerLimit;
	info2.m_upperLimit = &currentConstraintRow->m_upperLimit;
	info2.m_numIterations = infoGlobal.m_numIterations;
	constraint->getInfo2(&info2);

	///finalize the constraint setup
	for (int j = 0; j < info1.m_numConstraintRows; j++)
	{
		btSolverConstraint& solverConstraint = currentConstraintRow[j];

		if (solverConstraint.m_upperLimit >= constraint->getBreakingImpulseThreshold())
		{
			solverConstraint.m_upperLimit = constraint->getBreakingImpulseThreshold();
		}

		if (solverConstraint.m_lowerLimit <= -constraint->getBreakingImpulseThreshold())
		{
			solverConstraint.m_lowerLimit = -constraint->getBreakingImpulseThreshold();
		}

		solverConstraint.m_originalContactPoint = constraint;
		solverConstraint.m_rhs = 0;
		solverConstraint.m_rhsPenetration = 0;
	}
}

void btSmallStepPGSConstraintSolver::convertBodies(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& infoGlobal)
{
	BT_PROFILE("convertBodies");
	for (int i = 0; i < numBodies; i++)
	{
		bodies[i]->setCompanionId(-1);
	}
#if BT_THREADSAFE
	m_kinematicBodyUniqueIdToSolverBodyTable.resize(0);
#endif  // BT_THREADSAFE

	m_tmpSolverBodyPool.reserve(numBodies + 1);
	m_tmpSolverBodyPool.resize(0);

	//btSolverBody& fixedBody = m_tmpSolverBodyPool.expand();
	//initSolverBody(&fixedBody,0);

	for (int i = 0; i < numBodies; i++)
	{
		int bodyId = getOrInitSolverBody(*bodies[i], m_subTimeStep);
	}
}

void btSmallStepPGSConstraintSolver::updateConstraints(int iteration, btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	BT_PROFILE("updateConstraints");
	{
		//btTypedConstraint::btConstraintInfo1 tmp_info;
		int currentRow = 0;
		for (int i = 0; i < numConstraints; i++)
		{
			btTypedConstraint::btConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];

			if (info1.m_numConstraintRows)
			{
				btSolverConstraint* currentConstraintRow = &m_tmpSolverNonContactConstraintPool[currentRow];

				btTypedConstraint* constraint = constraints[i];

				int solverBodyIdA = currentConstraintRow->m_solverBodyIdA;
				int solverBodyIdB = currentConstraintRow->m_solverBodyIdB;

				updateJoint(iteration, currentConstraintRow, constraint, info1, solverBodyIdA, solverBodyIdB, infoGlobal);
			}

			currentRow += info1.m_numConstraintRows;
		}
	}

	{
		int numContactConstraits  = m_tmpSolverContactConstraintPool.size();
		for (int i = 0; i < numContactConstraits; i++)
		{
			btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[i];
			updateContact(iteration, solveManifold, infoGlobal);
		}
	}
}

void btSmallStepPGSConstraintSolver::updateJoint(int iteration, btSolverConstraint* currentConstraintRow, btTypedConstraint* constraint, const btTypedConstraint::btConstraintInfo1& info1, int solverBodyIdA, int solverBodyIdB, const btContactSolverInfo& infoGlobal)
{
	const btRigidBody& rbA = constraint->getRigidBodyA();
	const btRigidBody& rbB = constraint->getRigidBodyB();

	const btSolverBody* bodyAPtr = &m_tmpSolverBodyPool[solverBodyIdA];
	const btSolverBody* bodyBPtr = &m_tmpSolverBodyPool[solverBodyIdB];

	btTypedConstraint::btConstraintInfo2 info2;
	info2.fps = m_invSubTimeStep;
	info2.erp = infoGlobal.m_erp;
	info2.m_J1linearAxis = currentConstraintRow->m_contactNormal1;
	info2.m_J1angularAxis = currentConstraintRow->m_relpos1CrossNormal;
	info2.m_J2linearAxis = currentConstraintRow->m_contactNormal2;
	info2.m_J2angularAxis = currentConstraintRow->m_relpos2CrossNormal;
	info2.rowskip = sizeof(btSolverConstraint) / sizeof(btScalar);  //check this
																	///the size of btSolverConstraint needs be a multiple of btScalar
	btAssert(info2.rowskip * sizeof(btScalar) == sizeof(btSolverConstraint));
	info2.m_constraintError = &currentConstraintRow->m_rhs;
	currentConstraintRow->m_cfm = infoGlobal.m_globalCfm;
	info2.m_damping = infoGlobal.m_damping;
	info2.cfm = &currentConstraintRow->m_cfm;
	info2.m_lowerLimit = &currentConstraintRow->m_lowerLimit;
	info2.m_upperLimit = &currentConstraintRow->m_upperLimit;
	info2.m_numIterations = infoGlobal.m_numIterations;
	constraint->getInfo2(&info2);

	for (int j = 0; j < info1.m_numConstraintRows; j++)
	{
		btSolverConstraint& solverConstraint = currentConstraintRow[j];
		{
			const btVector3& ftorqueAxis1 = solverConstraint.m_relpos1CrossNormal;
			solverConstraint.m_angularComponentA = constraint->getRigidBodyA().getInvInertiaTensorWorld() * ftorqueAxis1 * constraint->getRigidBodyA().getAngularFactor();
		}
		{
			const btVector3& ftorqueAxis2 = solverConstraint.m_relpos2CrossNormal;
			solverConstraint.m_angularComponentB = constraint->getRigidBodyB().getInvInertiaTensorWorld() * ftorqueAxis2 * constraint->getRigidBodyB().getAngularFactor();
		}

		{
			btVector3 iMJlA = solverConstraint.m_contactNormal1 * rbA.getInvMass();
			btVector3 iMJaA = rbA.getInvInertiaTensorWorld() * solverConstraint.m_relpos1CrossNormal;
			btVector3 iMJlB = solverConstraint.m_contactNormal2 * rbB.getInvMass();  //sign of normal?
			btVector3 iMJaB = rbB.getInvInertiaTensorWorld() * solverConstraint.m_relpos2CrossNormal;

			btScalar sum = iMJlA.dot(solverConstraint.m_contactNormal1);
			sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
			sum += iMJlB.dot(solverConstraint.m_contactNormal2);
			sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);
			btScalar fsum = btFabs(sum);
			btAssert(fsum > SIMD_EPSILON);
			btScalar sorRelaxation = 1.f;  //todo: get from globalInfo?
			solverConstraint.m_jacDiagABInv = fsum > SIMD_EPSILON ? sorRelaxation / sum : 0.f;
		}

		{
			btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(bodyAPtr->m_linearVelocity) + solverConstraint.m_relpos1CrossNormal.dot(bodyAPtr->m_angularVelocity);
			btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(bodyBPtr->m_linearVelocity) + solverConstraint.m_relpos2CrossNormal.dot(bodyBPtr->m_angularVelocity);

			btScalar rel_vel = vel1Dotn + vel2Dotn;
			btScalar positionalError = solverConstraint.m_rhs;  //already filled in by getConstraintInfo2
			btScalar velocityError = -rel_vel * info2.m_damping;
			btScalar penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
			btScalar velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
			solverConstraint.m_rhs = velocityImpulse + penetrationImpulse;
		}
	}
}

void btSmallStepPGSConstraintSolver::updateContact(int iteration, btSolverConstraint& solverConstraint, const btContactSolverInfo& infoGlobal)
{
	int solverBodyIdA = solverConstraint.m_solverBodyIdA;
	int solverBodyIdB = solverConstraint.m_solverBodyIdB;

	btSolverBody& solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody& solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];

	btRigidBody* rb0 = solverBodyA.m_originalBody;
	btRigidBody* rb1 = solverBodyB.m_originalBody;

	btManifoldPoint* cp = (btManifoldPoint*)solverConstraint.m_originalContactPoint;

	btVector3 normalWorldOnB = cp->m_normalWorldOnB;

	btVector3 pos1, pos2;
	btScalar distance = 0;
	if (!rb0)
	{
		pos2 = solverBodyB.m_worldTransform(cp->m_localPointB);

		distance = normalWorldOnB.dot(cp->getPositionWorldOnA()) - normalWorldOnB.dot(pos2);
		pos1 = pos2 + distance * normalWorldOnB;
	}
	else if (!rb1)
	{
		pos1 = solverBodyA.m_worldTransform(cp->m_localPointA);

		distance = normalWorldOnB.dot(pos1) - normalWorldOnB.dot(cp->getPositionWorldOnB());
		pos2 = pos1 - distance * normalWorldOnB;
	}
	else
	{
		pos1 = solverBodyA.m_worldTransform(cp->m_localPointA);
		pos2 = solverBodyB.m_worldTransform(cp->m_localPointB);

		distance = normalWorldOnB.dot(pos1) - normalWorldOnB.dot(pos2);
	}

	const btVector3 rel_pos1 = pos1 - solverBodyA.m_worldTransform.getOrigin();
	const btVector3 rel_pos2 = pos2 - solverBodyB.m_worldTransform.getOrigin();

	if (rb0)
	{
		btVector3 torqueAxis0 = rel_pos1.cross(solverConstraint.m_contactNormal1);
		solverConstraint.m_relpos1CrossNormal = torqueAxis0;
		solverConstraint.m_angularComponentA = rb0->getInvInertiaTensorWorld() * torqueAxis0 * rb0->getAngularFactor();
	}
	if (rb1)
	{
		btVector3 torqueAxis1 = rel_pos2.cross(solverConstraint.m_contactNormal2);
		solverConstraint.m_relpos2CrossNormal = torqueAxis1;
		solverConstraint.m_angularComponentB = rb1->getInvInertiaTensorWorld() * torqueAxis1 * rb1->getAngularFactor();
	}

	btScalar cfm = solverConstraint.m_cfm / solverConstraint.m_jacDiagABInv;

	{
#ifdef COMPUTE_IMPULSE_DENOM
		btScalar denom0 = rb0->computeImpulseDenominator(pos1, cp.m_normalWorldOnB);
		btScalar denom1 = rb1->computeImpulseDenominator(pos2, cp.m_normalWorldOnB);
#else
		btVector3 vec;
		btScalar denom0 = 0.f;
		btScalar denom1 = 0.f;
		if (rb0)
		{
			vec = (solverConstraint.m_angularComponentA).cross(rel_pos1);
			denom0 = rb0->getInvMass() + normalWorldOnB.dot(vec);
		}
		if (rb1)
		{
			vec = (solverConstraint.m_angularComponentB).cross(rel_pos2);
			denom1 = rb1->getInvMass() - normalWorldOnB.dot(vec);
		}
#endif  //COMPUTE_IMPULSE_DENOM

		btScalar denom = infoGlobal.m_sor / (denom0 + denom1 + cfm);
		solverConstraint.m_jacDiagABInv = denom;
	}

	btScalar elapsedTime = iteration * m_subTimeStep;
	btScalar penetration = distance - elapsedTime * solverConstraint.m_targetVel + infoGlobal.m_linearSlop;
	penetration += btMax(btScalar(0), cp->getDistance());

	{
		btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(solverBodyA.m_linearVelocity) + solverConstraint.m_relpos1CrossNormal.dot(solverBodyA.m_angularVelocity);
		btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(solverBodyB.m_linearVelocity) + solverConstraint.m_relpos2CrossNormal.dot(solverBodyB.m_angularVelocity);

		btScalar rel_vel = vel1Dotn + vel2Dotn;

		btScalar velocityError = solverConstraint.m_targetVel - rel_vel;
		btScalar positionalError = -penetration * solverConstraint.m_erp * m_invSubTimeStep;

		positionalError = btMin(positionalError, infoGlobal.m_maxContactConstraintStablizationSpeed);

		btScalar penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
		btScalar velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
		
		solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
		solverConstraint.m_cfm = cfm * solverConstraint.m_jacDiagABInv;
	}

	btSolverConstraint& frictionConstraint = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex];
	updateFrictionConstraint(iteration, frictionConstraint,
							 solverBodyIdA, solverBodyIdB, *cp, rel_pos1, rel_pos2, infoGlobal.m_sor, infoGlobal);

	if (infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS)
	{
		btSolverConstraint& frictionConstraint2 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex + 1];
		updateFrictionConstraint(iteration, frictionConstraint2,
								 solverBodyIdA, solverBodyIdB, *cp, rel_pos1, rel_pos2, infoGlobal.m_sor, infoGlobal);
	}
}

void btSmallStepPGSConstraintSolver::updateFrictionConstraint(int iteration, btSolverConstraint& solverConstraint,
															  int solverBodyIdA, int solverBodyIdB,
															  const btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2,
															  btScalar relaxation, const btContactSolverInfo& infoGlobal)
{
	btSolverBody& solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody& solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];

	btRigidBody* bodyA = m_tmpSolverBodyPool[solverBodyIdA].m_originalBody;
	btRigidBody* bodyB = m_tmpSolverBodyPool[solverBodyIdB].m_originalBody;

	if (bodyA)
	{
		btVector3 ftorqueAxis1 = rel_pos1.cross(solverConstraint.m_contactNormal1);
		solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentA = bodyA->getInvInertiaTensorWorld() * ftorqueAxis1 * bodyA->getAngularFactor();
	}

	if (bodyB)
	{
		btVector3 ftorqueAxis1 = rel_pos2.cross(solverConstraint.m_contactNormal2);
		solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentB = bodyB->getInvInertiaTensorWorld() * ftorqueAxis1 * bodyB->getAngularFactor();
	}

	{
		btVector3 vec;
		btScalar denom0 = 0.f;
		btScalar denom1 = 0.f;
		if (bodyA)
		{
			vec = (solverConstraint.m_angularComponentA).cross(rel_pos1);
			denom0 = bodyA->getInvMass() + solverConstraint.m_contactNormal1.dot(vec);
		}
		if (bodyB)
		{
			vec = (solverConstraint.m_angularComponentB).cross(rel_pos2);
			denom1 = bodyB->getInvMass() + solverConstraint.m_contactNormal2.dot(vec);
		}
		btScalar denom = relaxation / (denom0 + denom1);
		solverConstraint.m_jacDiagABInv = denom;
	}

	{
		btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(solverBodyA.m_linearVelocity) + solverConstraint.m_relpos1CrossNormal.dot(solverBodyA.m_angularVelocity);
		btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(solverBodyB.m_linearVelocity) + solverConstraint.m_relpos2CrossNormal.dot(solverBodyB.m_angularVelocity);

		btScalar rel_vel = vel1Dotn + vel2Dotn;

		btScalar velocityError = solverConstraint.m_targetVel - rel_vel;
		btScalar velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;

		solverConstraint.m_rhs = velocityImpulse;
	}
}

void btSmallStepPGSConstraintSolver::integrateBodies(int iBegin, int iEnd, btScalar timeStep, const btContactSolverInfo& infoGlobal)
{
	BT_PROFILE("integrateBodies");
	for (int i = iBegin; i < iEnd; i++) 
	{
		btSolverBody& solverBody = m_tmpSolverBodyPool[i];
		btRigidBody* body = solverBody.m_originalBody;
		if (body)
		{
			btTransform originalTransform = solverBody.m_worldTransform;

			solverBody.updateVelocityAndTransform(timeStep);

			body->setLinearVelocity(solverBody.m_linearVelocity);
			body->setAngularVelocity(solverBody.m_angularVelocity);
			body->setWorldTransform(solverBody.m_worldTransform);
			body->updateInertiaTensor();
		}

		solverBody.m_deltaLinearVelocity.setZero();
		solverBody.m_deltaAngularVelocity.setZero();
	}
}

void btSmallStepPGSConstraintSolver::writeBackJoints(int iBegin, int iEnd, const btContactSolverInfo& infoGlobal)
{
	for (int j = iBegin; j < iEnd; j++)
	{
		const btSolverConstraint& solverConstr = m_tmpSolverNonContactConstraintPool[j];
		btTypedConstraint* constr = (btTypedConstraint*)solverConstr.m_originalContactPoint;

		constr->internalSetAppliedImpulse(solverConstr.m_appliedImpulse);
		if (btFabs(solverConstr.m_appliedImpulse) >= constr->getBreakingImpulseThreshold())
		{
			constr->setEnabled(false);
		}
	}
}

void btSmallStepPGSConstraintSolver::updateJointsFeedback(int iBegin, int iEnd, const btContactSolverInfo& infoGlobal)
{
	for (int j = iBegin; j < iEnd; j++)
	{
		const btSolverConstraint& solverConstr = m_tmpSolverNonContactConstraintPool[j];
		btTypedConstraint* constr = (btTypedConstraint*)solverConstr.m_originalContactPoint;
		btJointFeedback* fb = constr->getJointFeedback();
		btScalar preAppliedImpulse = m_tmpNonContactConstraintsPreAppliedImpulse[j];
		if (fb)
		{
			btScalar deltaImpulse = solverConstr.m_appliedImpulse - preAppliedImpulse;
			btScalar deltaForce = deltaImpulse * m_invSubTimeStep;

			fb->m_appliedForceBodyA += solverConstr.m_contactNormal1 * constr->getRigidBodyA().getLinearFactor() * deltaForce;
			fb->m_appliedForceBodyB += solverConstr.m_contactNormal2 * constr->getRigidBodyB().getLinearFactor() * deltaForce;
			fb->m_appliedTorqueBodyA += solverConstr.m_relpos1CrossNormal * constr->getRigidBodyA().getAngularFactor() * deltaForce;
			fb->m_appliedTorqueBodyB += solverConstr.m_relpos2CrossNormal * constr->getRigidBodyB().getAngularFactor() * deltaForce; /*RGM ???? */
		}
		m_tmpNonContactConstraintsPreAppliedImpulse[j] = solverConstr.m_appliedImpulse;
	}
}

void btSmallStepPGSConstraintSolver::applySplitImpulses(int iBegin, int iEnd, btScalar timeStep, const btContactSolverInfo& infoGlobal)
{
	for (int i = iBegin; i < iEnd; i++)
	{
		btSolverBody& solverBody = m_tmpSolverBodyPool[i];
		btRigidBody* body = solverBody.m_originalBody;
		if (body)
		{
			btTransform originalTransform = solverBody.m_worldTransform;

			btTransform newTransform;
			btTransformUtil::integrateTransform(solverBody.m_worldTransform, solverBody.m_pushVelocity, 
				solverBody.m_turnVelocity * infoGlobal.m_splitImpulseTurnErp, timeStep, 
				newTransform);

			solverBody.m_worldTransform = newTransform;
			body->setWorldTransform(newTransform);
			body->updateInertiaTensor();
		}

		solverBody.m_pushVelocity.setZero();
		solverBody.m_turnVelocity.setZero();
	}
}

void btSmallStepPGSConstraintSolver::applyExternalImpulses(int iBegin, int iEnd, const btContactSolverInfo& infoGlobal)
{
	for (int i = iBegin; i < iEnd; i++)
	{
		btSolverBody& solverBody = m_tmpSolverBodyPool[i];
		btRigidBody* body = solverBody.m_originalBody;
		if (body)
		{
			solverBody.m_externalTorqueImpulse = body->getTotalTorque() * body->getInvInertiaTensorWorld() * m_subTimeStep;
			if (body->getFlags() & BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT)
			{
				btVector3 gyroForce = body->computeGyroscopicForceExplicit(infoGlobal.m_maxGyroscopicForce);
				solverBody.m_externalTorqueImpulse -= gyroForce * body->getInvInertiaTensorWorld() * m_subTimeStep;
			}
			else if (body->getFlags() & BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD)
			{
				solverBody.m_externalTorqueImpulse += body->computeGyroscopicImpulseImplicit_World(m_subTimeStep);
			}
			else if (body->getFlags() & BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY)
			{
				solverBody.m_externalTorqueImpulse += body->computeGyroscopicImpulseImplicit_Body(m_subTimeStep);
			}

			solverBody.m_linearVelocity += solverBody.m_externalForceImpulse;
			solverBody.m_angularVelocity += solverBody.m_externalTorqueImpulse;
		}
	}
}

void btSmallStepPGSConstraintSolver::writeBackBodies(int iBegin, int iEnd, const btContactSolverInfo& infoGlobal)
{
	for (int i = iBegin; i < iEnd; i++)
	{
		btSolverBody& solverBody = m_tmpSolverBodyPool[i];
		btRigidBody* body = solverBody.m_originalBody;
		if (body)
		{
			body->setCompanionId(-1);
		}
	}
}