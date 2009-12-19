INCLUDE_DIRECTORIES( ${PHYSICS_EFFECTS_SOURCE_DIR}  )

SET(PfxCommonFormat_SRCS
	PfxCommonExport.cpp
	PfxCommonFileIO.cpp
	PfxCommonFormat.cpp
	PfxCommonImport.cpp
)

SET(PfxCommonFormat_HDRS
	PfxCommonExport.h
	PfxCommonFileIO.h
	PfxCommonFormat.h
	PfxCommonImport.h
	PfxCommonTypes.h
)


ADD_LIBRARY(PfxCommonFormat ${PfxCommonFormat_SRCS} ${PfxCommonFormat_HDRS})

SET_TARGET_PROPERTIES(PfxCommonFormat PROPERTIES VERSION ${PHYSICS_EFFECTS_VERSION})
SET_TARGET_PROPERTIES(PfxCommonFormat PROPERTIES SOVERSION ${PHYSICS_EFFECTS_VERSION})
