
#include "physics_effects.h"
#include "physics_func.h"
#include "../common/perf_func.h"

static int frameCount = 0;
static int sceneId = 2;

int main()
{

	perf_init();
	physics_init();

	physics_create_scene(sceneId);

	frameCount = 0;

	//createScene();

	while(frameCount<600) {
		physics_simulate();
		perf_sync();
	}

	SCE_PFX_PRINTF("program complete\n");

	return 0;
}
