# HelloWorld is a minimal sample creating, stepping and deleting a Physics Effects dynamics world

INCLUDE_DIRECTORIES(
${PHYSICS_EFFECTS_SOURCE_DIR}
)

LINK_LIBRARIES(
  PhysicsEffectsRigidBody PhysicsEffectsCollision
)

ADD_EXECUTABLE(AppHelloWorld
	main.cpp 
)

