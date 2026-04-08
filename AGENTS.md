# AGENTS.md

## Cursor Cloud specific instructions

### Overview

This is **OpenSiv3D v0.6.16**, a C++20 creative coding framework. The Linux build uses CMake + Ninja and compiles a static library (`libSiv3D.a`) linked into user applications.

### Build toolchain

- **Compiler**: Use Clang 18 with `--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/13` to resolve C++ standard library header lookup. The default `cc`/`c++` alternatives point to Clang, but Clang selects a GCC 14 installation directory whose C++ headers are absent. Passing this flag forces Clang to use GCC 13 headers.
- **Alternative**: GCC 11 (`gcc-11`/`g++-11`) matches the upstream CI (Ubuntu 22.04), but a SIMDE/Boost header conflict requires the `Platform.hpp` fix described below.
- A `libstdc++.so` symlink at `/usr/lib/x86_64-linux-gnu/libstdc++.so` is needed for Clang linking. Create it with: `sudo ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/x86_64-linux-gnu/libstdc++.so`

### Known issue: SIMDE / Boost 1.83 AVX512 conflict

The upstream CI targets Ubuntu 22.04 (Boost 1.74). On Ubuntu 24.04 (Boost 1.83), Boost's `cpp_int/intel_intrinsics.hpp` includes `<immintrin.h>` which defines AVX512 intrinsics that collide with SIMDE's `SIMDE_ENABLE_NATIVE_ALIASES` macros (`_mm_loadu_epi8`, etc.). The root cause: `CMakeLists.txt` uses `-msse4.1`, but `Platform.hpp` checks `__SSE4_2__` for its SSE intrinsic flag, so the code falls into the SIMDE branch on SSE4.1-only builds.

**Fix**: In `Siv3D/include/Siv3D/Platform.hpp`, the Linux SSE detection was changed from `__SSE4_2__` to `__SSE4_1__ || __SSE4_2__` so that `-msse4.1` correctly enables native SSE intrinsics and avoids SIMDE alias conflicts with Boost.

### Build & test commands

See `.github/workflows/ubuntu.yml` for the canonical build steps. Summary for Clang on this VM:

```bash
# 1. Build Siv3D library
cd Linux && rm -rf build && mkdir build && cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_CXX_FLAGS="--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/13" \
  -DCMAKE_C_FLAGS="--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/13" ..
cmake --build .

# 2. Install
sudo cmake --install build   # from Linux/

# 3. Build test app
cd ../App && rm -rf build && mkdir build && cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_CXX_FLAGS="--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/13" ..
cmake --build .

# 4. Run test
cd .. && ./Siv3DTest

# 5. CTest
cd build && cmake -DBUILD_TESTING:BOOL=ON . && ctest --output-on-failure --verbose
```

### Headless mode

The default `Linux/App/Main.cpp` uses `SIV3D_SET(EngineOption::Renderer::Headless)` for non-graphical mode, which works on VMs without a display/GPU. ALSA and JACK warnings are expected on headless environments and are harmless (audio falls back to NoSound backend).

### Lint

This project has no dedicated lint tooling. The compiler warnings (`-Wall -Wextra`) serve as the primary code quality check during build.
