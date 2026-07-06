"""
PlatformIO pre-build script: patch HuskyLens library for ESP32.

The huskylens/HuskyLens library calls max()/min() with mismatched
argument types (int16_t vs int) inside HUSKYLENS.h. ESP32's toolchain
resolves std::max/std::min via strict templates and refuses to deduce
a common type, so the build fails with:

  error: no matching function for call to 'max(int16_t&, int)'
  error: no matching function for call to 'min(int16_t&, int&)'

This script casts the ambiguous arguments to int so overload
resolution succeeds. It re-runs on every build but only writes the
file if a change is actually needed, so it's safe to run repeatedly
and survives `pio run -t clean` / library reinstalls.
"""

Import("env")
import os

def patch_huskylens():
    libdeps_dir = env.subst("$PROJECT_LIBDEPS_DIR")
    pioenv = env.subst("$PIOENV")
    lib_path = os.path.join(libdeps_dir, pioenv, "HuskyLens", "HUSKYLENS.h")

    if not os.path.isfile(lib_path):
        print(f"[patch_huskylens] {lib_path} not found yet, skipping (will patch on next run).")
        return

    with open(lib_path, "r") as f:
        content = f.read()

    original = content

    content = content.replace(
        "max(protocolInfo.protocolSize, 1)",
        "max((int)protocolInfo.protocolSize, 1)",
    )
    content = content.replace(
        "currentIndex = min(currentIndex, result);",
        "currentIndex = min((int)currentIndex, result);",
    )

    if content != original:
        with open(lib_path, "w") as f:
            f.write(content)
        print(f"[patch_huskylens] Patched {lib_path} (min/max type mismatch fixed).")
    else:
        print(f"[patch_huskylens] {lib_path} already patched or pattern not found, no changes made.")

patch_huskylens()