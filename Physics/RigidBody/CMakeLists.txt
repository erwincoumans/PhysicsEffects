INCLUDE_DIRECTORIES( ${PHYSICS_EFFECTS_SOURCE_DIR}  )

SET(PhysicsEffectsRigidBody_SRCS
	Mass.cpp
	RigidBodies.cpp
	RigidBodiesPPU.cpp
	Snapshot.cpp
)

SET(PhysicsEffectsRigidBody_HDRS
Mass.h
RigidBodies.h
)


ADD_LIBRARY(PhysicsEffectsRigidBody ${PhysicsEffectsRigidBody_SRCS} ${PhysicsEffectsRigidBody_HDRS})

SET_TARGET_PROPERTIES(PhysicsEffectsRigidBody PROPERTIES VERSION ${PHYSICS_EFFECTS_VERSION})
SET_TARGET_PROPERTIES(PhysicsEffectsRigidBody PROPERTIES SOVERSION ${PHYSICS_EFFECTS_VERSION})

