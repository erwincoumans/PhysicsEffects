INCLUDE_DIRECTORIES( ${PHYSICS_EFFECTS_SOURCE_DIR}  )

SET(PhysicsEffectsRayCast_SRCS
RayCast.cpp
)

SET(PhysicsEffectsRayCast_HDRS
RayCast.h
)


ADD_LIBRARY(PhysicsEffectsRayCast ${PhysicsEffectsRayCast_SRCS} ${PhysicsEffectsRayCast_HDRS})

SET_TARGET_PROPERTIES(PhysicsEffectsRayCast PROPERTIES VERSION ${PHYSICS_EFFECTS_VERSION})
SET_TARGET_PROPERTIES(PhysicsEffectsRayCast PROPERTIES SOVERSION ${PHYSICS_EFFECTS_VERSION})

