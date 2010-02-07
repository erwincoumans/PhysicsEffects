# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# This is the variable for Windows.  I use this to define the root of my directory structure.


# You shouldn't have to modify anything below this line 
########################################################
SET(GLUT_ROOT ${BULLET_PHYSICS_SOURCE_DIR}/Glut)

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL  ${PHYSICS_EFFECTS_SOURCE_DIR}
)



if (WIN32)
LINK_LIBRARIES(
OpenGLSupport  BulletMultiThreaded BulletDynamics  BulletCollision LinearMath   PhysicsEffectsRigidBody  PhysicsEffectsCollision ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

	ADD_EXECUTABLE(AppPfxBackend
		WIN32
		PfxBackendDemo.cpp
		PfxBackendDemo.h
		btPhysicsEffectsWorld.cpp
		btPhysicsEffectsWorld.h
		../OpenGL/Win32AppMain.cpp
		../OpenGL/Win32DemoApplication.cpp
	)
else (WIN32)


LINK_LIBRARIES(
OpenGLSupport BulletMultiThreaded  BulletDynamics  BulletCollision LinearMath   PhysicsEffectsRigidBody PhysicsEffectsCollision  ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)


	ADD_EXECUTABLE(AppPfxBackend
		main.cpp
		PfxBackendDemo.cpp 
		PfxBackendDemo.h
	)
endif (WIN32)
