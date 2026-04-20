# Portal_solver — AI / new machine setup guide

Paste this file (or its path) into the chat when opening this repository on a **new computer** so the assistant knows **exactly** what is required to build, run, and debug the project.

## What this project is

- **Qt desktop app** (C++17): steel portal frame with A-truss, **TS 498**-style STR load combinations, **OpenSees** elastic static analysis (external process, **not** linked as a C++ library).
- **CMake** generates the Visual Studio / Ninja build; the app shells out to **OpenSees.exe** for Tcl scripts.

## Mandatory toolchain

| Requirement | Notes |
|-------------|--------|
| **CMake** | ≥ **3.21** (`cmake_minimum_required` in root `CMakeLists.txt`). |
| **C++ compiler** | **C++17**. On Windows: **MSVC** (Visual Studio 2022 recommended) is what the repo is typically built with. |
| **Qt** | **Qt6** or **Qt5** — components **`Widgets`** and **`Concurrent`** (`find_package` in `CMakeLists.txt`). Set `CMAKE_PREFIX_PATH` to the Qt installation if CMake cannot find Qt. |
| **Git** | For clone; optional network for first-time OpenSees zip fetch (Windows). |

Optional CMake cache variables (see `CMakeLists.txt`):

- `OPENSEES_INCLUDE_DIR`, `OPENSEES_LIBRARY_DIR` — reserved for a **future** in-process OpenSees link; the **current** app does **not** require them to run analysis.

## OpenSees (runtime — critical)

Analysis calls **OpenSees** as a **subprocess** (`OpenSeesRunner.cpp`).

### Resolution order for the executable

1. Environment variable **`OPENSEES_EXE`** — full path to `OpenSees.exe` (Windows) or `OpenSees` (Unix). Must exist on disk if set.
2. Else **`OpenSees.exe`** next to **`PortalSolver.exe`** (application directory).
3. Else bare command **`OpenSees`** (must be on **PATH**).

### Windows — recommended bundle (CMake)

`cmake/PortalOpenSees.cmake`:

- If **`third_party/opensees-berkeley/OpenSees3.8.0/bin/OpenSees.exe`** exists, **POST_BUILD** copies **`OpenSees.exe`**, **`libiomp5md.dll`**, and the **`lib/`** tree (Tcl) next to the built **`PortalSolver.exe`**.
- If missing and **`PORTAL_FETCH_BERKELEY_OPENSEES=ON`** (default on Windows), CMake may **download** Berkeley’s **OpenSees 3.8.0 x64** zip into **`third_party/cache/``** and expand it under **`third_party/opensees-berkeley/`** (needs **network** once).
- Override with **`OPENSEES_EXE_TO_BUNDLE`** (CMake `FILEPATH`) pointing to a specific `OpenSees.exe` to copy beside the app.

### Tcl / init failure (Windows)

If OpenSees starts but Tcl fails (`init.tcl` not found), the runner sets **`TCL_LIBRARY`** from the executable’s directory when **`lib/tcl8.6`** exists beside the bundle (see `OpenSeesRunner.cpp`). Ensure the **`lib`** folder from the Berkeley package is deployed **with** the exe.

### Linux / macOS

- There is **no** automatic zip fetch like Windows; install OpenSees from your distro or build from source, then either put the binary on **PATH** or set **`OPENSEES_EXE`**.
- Copy **`lib/tcl8.6`** next to the binary if your OpenSees build expects it the same way as Berkeley Windows layout.

## Build steps (generic)

```text
git clone <repo-url>
cd Portal_solver
cmake -S . -B build -DCMAKE_PREFIX_PATH=<path-to-Qt>/lib/cmake/Qt6
cmake --build build --config Release
```

On Windows with Visual Studio generator, open the generated `.sln` or use `--config Release` as above. The output **`PortalSolver.exe`** is usually under **`build/Release/`** (single-config generators may use `build/` only).

**windeployqt** runs as a **POST_BUILD** step when found so Qt DLLs sit next to the exe.

## Repository layout hints for AI

- **`PortalSolver.*`** — geometry, validation, orchestrates combos + OpenSees passes.
- **`OpenSeesRunner.*`** — writes Tcl, runs OpenSees, parses displacements / element forces.
- **`TurkishLoads.*`** — characteristic line loads and STR combinations.
- **`SectionOptimizer.*`** — EC3-style checks and steel catalog picks.
- **`SteelCatalog.*`** — rolled I and double-angle tables.
- **`third_party/`** — optional OpenSees Berkeley bundle (may be gitignored partially; large zips often in `third_party/cache/`).

## What to tell the AI in one paragraph (copy-paste)

> This is a **CMake 3.21+**, **C++17**, **Qt (Widgets + Concurrent)** desktop app on Windows (MSVC). It does **not** link OpenSees as a library; it runs **`OpenSees.exe`** via `QProcess`. On Windows, configure/build so **`cmake/PortalOpenSees.cmake`** can bundle Berkeley **OpenSees 3.8.0** into `third_party/opensees-berkeley/` (or set **`OPENSEES_EXE`** / **`OPENSEES_EXE_TO_BUNDLE`**). The exe must ship with **`lib/tcl8.6`** and **`libiomp5md.dll`** when using the Berkeley layout. Set **`CMAKE_PREFIX_PATH`** to Qt if `find_package` fails. After build, **`PortalSolver.exe`** and OpenSees live in the same output folder.

## Optional checks after first successful build

- Run **`PortalSolver.exe`** from its output directory (not only from IDE) so **relative** OpenSees discovery matches production.
- Temporarily set **`OPENSEES_EXE`** to a known-good binary to rule out bundling issues.

---

*Maintainers: keep this file updated when CMake minimum version, Qt components, or OpenSees integration strategy changes.*
