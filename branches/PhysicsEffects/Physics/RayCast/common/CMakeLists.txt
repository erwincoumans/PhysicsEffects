
INCLUDE_DIRECTORIES( ${PHYSICS_EFFECTS_SOURCE_DIR}  )

SET(PhysicsEffectsRayIntersect_SRCS
RayIntersectHeightField.cpp
RayIntersect.cpp
)

SET(PhysicsEffectsRayIntersect_HDRS
RayIntersectHeightField.h
RayIntersect.h
RayCastConfig.h
Ray.h
RayCastIO.h
)


ADD_LIBRARY(PhysicsEffectsRayIntersect ${PhysicsEffectsRayIntersect_SRCS} ${PhysicsEffectsRayIntersect_HDRS})

SET_TARGET_PROPERTIES(PhysicsEffectsRayIntersect PROPERTIES VERSION ${PHYSICS_EFFECTS_VERSION})
SET_TARGET_PROPERTIES(PhysicsEffectsRayIntersect PROPERTIES SOVERSION ${PHYSICS_EFFECTS_VERSION})
