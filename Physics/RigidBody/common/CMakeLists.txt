INCLUDE_DIRECTORIES( ${PHYSICS_EFFECTS_SOURCE_DIR}  )

SET(PhysicsEffectsCollision_SRCS
	boxBoxDistance.cpp
	boxCapsuleDistance.cpp
	boxSphereDistance.cpp
	capsuleCapsuleDistance.cpp
	capsuleSphereDistance.cpp
	closestContact.cpp
	closestContactConvex.cpp
	closestContactHeightField.cpp
	closestContactLargeMesh.cpp
	CollObject.cpp
	Contact.cpp
	contactHeightField.cpp
	contactSolver.cpp
	convexBoxDistance.cpp
	convexCapsuleDistance.cpp
	convexConvexDistance.cpp
	convexSphereDistance.cpp
	HeightField.cpp
	HeightFieldFunction.cpp
	jointSolver.cpp
	PfxContactCache.cpp
	PfxGJKSolver.cpp
	PfxGJKSupportFunc.cpp
	PfxSimplexSolver.cpp
	trianglesBoxContacts.cpp
	trianglesCapsuleContacts.cpp
	trianglesConvexContacts.cpp
	trianglesSphereContacts.cpp
)

SET(PhysicsEffectsCollision_HDRS
	Box.h
	boxBoxDistance.h
	boxCapsuleDistance.h
	boxSphereDistance.h
	Capsule.h
	capsuleCapsuleDistance.h
	capsuleSphereDistance.h
	closestContact.h
	collidableTable.h
	CollObject.h
	CollPrim.h
	ConstraintCache.h
	ConstraintRowSolver.h
	Contact.h
	contactHeightField.h
	contactSolver.h
	convexBoxDistance.h
	convexCapsuleDistance.h
	convexConvexDistance.h
	convexSphereDistance.h
	Forces.h
	HeightField.h
	HeightFieldFunction.h
	intersectFunction.h
	Joint.h
	jointSolver.h
	ParallelGroup.h
	PfxContactCache.h
	PfxGJKSolver.h
	PfxGJKSupportFunc.h
	PfxSimplexSolver.h
	RigidBodyConfig.h
	RigidBodyIO.h
	sat_mesh_utils.h
	solverFuncTable.h
	Sphere.h
	sphereSphereDistance.h
	Spring.h
	SubData.h
	TrbDynBody.h
	TrbStateVec.h
	trianglesBoxContacts.h
	trianglesCapsuleContacts.h
	trianglesConvexContacts.h
	trianglesSphereContacts.h
	TriMesh.h
	vec_utils.h
)


ADD_LIBRARY(PhysicsEffectsCollision ${PhysicsEffectsCollision_SRCS} ${PhysicsEffectsCollision_HDRS})

SET_TARGET_PROPERTIES(PhysicsEffectsCollision PROPERTIES VERSION ${PHYSICS_EFFECTS_VERSION})
SET_TARGET_PROPERTIES(PhysicsEffectsCollision PROPERTIES SOVERSION ${PHYSICS_EFFECTS_VERSION})

