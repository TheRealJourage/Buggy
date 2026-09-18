Import("env")
import os

# Map each PlatformIO environment to its source directory
env_name = env["PIOENV"]

src_map = {
    "rp2040_slave": "slave",
    "uno_master": "master",
    "uno_master_debug": "master",
    "uno_bus_test": "test_uno",
    "rp2040_bus_test": "test_rp2040",
}

if env_name in src_map:
    src_dir = os.path.join(env.subst("$PROJECT_DIR"), src_map[env_name])
    env["PROJECT_SRC_DIR"] = src_dir
    env["PROJECTSRC_DIR"] = src_dir
