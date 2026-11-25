# PlatformIO pre-build script (all environments).
#
# Makes the same commit build the same binary on any machine (spec §10,
# Stage 0 acceptance):
#
# 1. FW_GIT_REV: short commit hash, with "-dirty" if the work tree has
#    uncommitted or untracked changes. Our code has no __DATE__/__TIME__.
# 2. SOURCE_DATE_EPOCH = the commit's timestamp. GCC uses it for __DATE__ and
#    __TIME__, which the Arduino core embeds (chip debug report); without it
#    every build differs by its build time.
# 3. Path prefix maps: strip the project, PlatformIO core and package
#    directories from __FILE__ and debug info, so the build doesn't depend
#    on where the checkout or ~/.platformio live.

import subprocess

Import("env")  # noqa: F821  (SCons global)


def git(project_dir, *args):
    return subprocess.check_output(
        ["git", *args], cwd=project_dir, stderr=subprocess.DEVNULL, text=True).strip()


def git_info(project_dir):
    try:
        rev = git(project_dir, "rev-parse", "--short=10", "HEAD")
        dirty = git(project_dir, "status", "--porcelain") != ""
        epoch = git(project_dir, "log", "-1", "--format=%ct", "HEAD")
        return rev + ("-dirty" if dirty else ""), epoch
    except (OSError, subprocess.CalledProcessError):
        return "unknown", "0"


project_dir = env.subst("$PROJECT_DIR")            # noqa: F821
packages_dir = env.subst("$PROJECT_PACKAGES_DIR")  # noqa: F821
core_dir = env.subst("$PROJECT_CORE_DIR")          # noqa: F821

rev, epoch = git_info(project_dir)
env.Append(CPPDEFINES=[("FW_GIT_REV", '\\"%s\\"' % rev)])  # noqa: F821
env["ENV"]["SOURCE_DATE_EPOCH"] = epoch                    # noqa: F821

prefix_maps = []
for path, repl in ((packages_dir, "/pio/packages"),
                   (core_dir, "/pio"),
                   (project_dir, ".")):
    for p in {path, path.replace("\\", "/")}:
        prefix_maps.append("-ffile-prefix-map=%s=%s" % (p, repl))
env.Append(CCFLAGS=prefix_maps)  # noqa: F821
