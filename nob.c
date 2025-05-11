#define NOB_IMPLEMENTATION
#include "nob.h" // Make sure nob.h is in your project directory

#include <stdio.h> // For snprintf
#include <errno.h> // For strerror with nob_copy_file

int main(int argc, char **argv) {
    NOB_GO_REBUILD_URSELF(argc, argv);

    // gcc build/main.c.o build/objects.c.o -o bouncing_ball_sim.exe -Llib -lraylib -lopengl32 -lgdi32 -lwinmm -mwindows
    Nob_Cmd cmd = {0};
    nob_cmd_append(&cmd, "gcc", "-Wall", "-Wextra");
    nob_cmd_append(&cmd, "-Iinclude", "-Llib");
    nob_cmd_append(&cmd, "-o", "bouncing_ball_sim.exe");
    nob_cmd_append(&cmd, "-O2");
    nob_cmd_append(&cmd, "src/main.c", "src/objects.c");
    nob_cmd_append(&cmd, "-lraylib", "-lopengl32", "-lgdi32", "-lwinmm");
    nob_cmd_append(&cmd, "-mwindows");
    if (!nob_cmd_run_sync(cmd)) return 1;
   
    return 0;
}
