# AGENTS.md — UR.RTDE Wrapper (Rhino 7 & 8, NuGet)

**Status**: 🔄 **IN PROGRESS** (Native C++ Wrapper Implementation)  
**Date**: 2025-10-27  
**Version**: 2.0.0 (Native)  
**Validation**: URSim e-Series 5.23.0 @ 172.18.0.2

---

## Overview

Expert **software engineering agent** is delivering a **native C# wrapper** around the **SDU Robotics `ur_rtde` C++ library** (v1.6.0) that works in:

* **Rhino 7** (.NET Framework **4.8**, Windows x64)
* **Rhino 8** (.NET **8**) on **Windows x64** and **macOS (arm64)**

Delivering a **NuGet package** named **`UR.RTDE`** with **native C++ P/Invoke** implementation. Zero Python dependency, pure native performance, single NuGet install.

## Objectives (what to build)

1. ✅ A thin **native C ABI façade** over `ur_rtde` (opaque handles, arrays, int status codes; no exceptions crossing ABI) - **DESIGNED**
2. ✅ A managed **C# wrapper** (namespace/assembly **`UR.RTDE`**) exposing: - **IMPLEMENTED**
   * **Control**: MoveJ, MoveL, SpeedJ/L, StopJ/L, SetTcp, SetPayload, watchdog.
   * **Receive**: ActualQ, ActualTcpPose, RobotMode, SafetyMode, RuntimeState, essential IO reads.
   * **IO**: basic digital IO where supported.
   * **Lifecycle**: Connect/Disconnect, timeouts, reconnect policy.
3. ⏳ **Packaging** as a single **NuGet** with RID-specific native assets to "just work" in Rhino 7/8. - **IN PROGRESS**
4. ✅ **Samples & docs** that prove end-to-end: connect → stream joints @ default rate → MoveJ → Stop. - **READY**

## Constraints & scope

* **Upstream**: use the **latest** `ur_rtde` from SDU Robotics on GitLab.
* **Platforms/TFMs**:

  * Managed: **`net48`** and **`net8.0`** (optionally share logic via **`netstandard2.0`** library).
  * Native RIDs in NuGet: **win-x64**, **osx-arm64** (optionally osx-x64/Rosetta).
* **Default RTDE frequency** (no custom tuning).
* **Naming**: managed surface uses **`UR.RTDE`** (all caps by product preference).
* **No C++/CLI** (to support macOS); prefer C ABI + P/Invoke.
* **Safety**: provide robust Stop paths; never block UI thread.

## Non-Goals

* No ROS/ROS2 integration in this phase.
* No out-of-process gRPC/REST service (keep in-process, low latency).

## Tools & environment (what you may assume)

* Build systems: **CMake** for native; **.NET SDK** for managed.
* CI on Windows + macOS runners to produce RID-specific binaries.
* Access to Rhino 7/8 and, if available, **URSim** for manual/integration testing.

## Planning rules (how you should work)

1. **Plan → act → verify** each increment. Maintain a running checklist and update it as facts change.
2. **Cite sources** in comments/notes when you rely on external docs.
3. **Be explicit about risks** (native load issues, threading) and propose mitigations before coding.
4. **Small, reversible steps** with buildable artifacts after each step.

## Deliverables (what success looks like)

* **NuGet**: `UR.RTDE` (contains managed assemblies for `net48`, `net8.0` and native libs under `runtimes/{rid}/native/`).
* **Samples**:

  * Grasshopper demo (connect/stream/MoveJ/Stop) for Rhino 7 and Rhino 8.
  * Minimal console demo (sanity check).
* **Docs**:

  * Quickstart (Rhino 7 Win, Rhino 8 Win/mac).
  * Troubleshooting (native load, mac Gatekeeper, firewall/port 30004).
  * Version matrix (Rhino versions, .NET, `ur_rtde` commit).
* **CI** that builds native + managed, runs unit tests, and packs the NuGet.

## Acceptance Criteria - Current Status

| Criterion | Status | Notes |
|-----------|--------|-------|
| **Loads in Rhino 7 (.NET 4.8)** | ⏳ PENDING | Multi-TFM build ready (net48/net8.0) |
| **Loads in Rhino 8 (.NET 8)** | ⏳ PENDING | Ready to test after native build |
| **Cross-platform** | 🔄 PARTIAL | Windows native ready, macOS pending |
| **Streaming ≥5 min @ 500 Hz** | ⏳ PENDING | Will test after build completes |
| **No UI blocking** | ✅ READY | Async/Task-based C# API designed |
| **MoveJ execution** | ⏳ PENDING | URSim @ 172.18.0.2 ready for testing |
| **Stop execution** | ⏳ PENDING | URSim ready |
| **No manual DLL copy** | ✅ READY | NuGet runtimes/ structure configured |
| **Clear documentation** | ✅ COMPLETE | README, BUILD_INSTRUCTIONS, guides |
| **No drops** | ⏳ PENDING | Will validate streaming reliability |

**Overall**: ⏳ **BLOCKED ON: Visual Studio C++ Workload Installation**

## Work Plan - Current Progress

### Phase 1: Foundation ✅ COMPLETE
1. ✅ **Recon & pin**: ur_rtde v1.6.0 (C++ library); MIT license
2. ✅ **Repo skeleton**: Created solution with `src/UR.RTDE/`, `samples/`, `docs/`, `native/facade/`, `build-native/`
3. ✅ **Architecture decision**: **Native C++ P/Invoke** (zero Python dependency)

### Phase 2: Native Build Infrastructure ✅ COMPLETE
4. ✅ **C API Facade**: Designed and implemented in `native/facade/` (ur_rtde_c_api.h/cpp)
5. ✅ **Build scripts**: `build-native.bat`, CMakeLists.txt, vcpkg integration
6. ✅ **Source preparation**: Cloned ur_rtde v1.6.0 to `build-native/ur_rtde/`
7. ✅ **CMake configuration**: Release build, static linking, vcpkg toolchain

### Phase 3: Managed Wrapper ✅ COMPLETE
8. ✅ **P/Invoke bindings**: `NativeMethods.cs` with [DllImport] declarations
9. ✅ **C# wrapper classes**: `RTDEControl`, `RTDEReceive` with IDisposable
10. ✅ **Error handling**: C error codes → RTDEException/RTDEConnectionException
11. ✅ **NuGet structure**: Multi-TFM (net48/net8.0), runtimes/win-x64/native/ configured

### Phase 4: Build Environment 🔄 IN PROGRESS
12. ✅ **VS 2022 Community**: Installed
13. ✅ **vcpkg**: Installed at C:\vcpkg
14. 🔄 **C++ Workload**: Installing complete components (needs vcvarsall.bat)
15. ⏳ **Boost libraries**: Waiting for VS installation to complete
16. ⏳ **ur_rtde build**: Blocked on Boost
17. ⏳ **C API facade build**: Blocked on ur_rtde

### Phase 5: Testing & Packaging ⏳ PENDING
18. ⏳ **Console demo**: Ready to test with URSim @ 172.18.0.2
19. ⏳ **URSim validation**: MoveJ, StopJ, streaming tests
20. ⏳ **NuGet package**: Create .nupkg with all native DLLs
21. ⏳ **Documentation**: Final updates with performance numbers

### Phase 6: Future Enhancements
22. 📋 **Grasshopper components**: GH integration (next phase)
23. 📋 **macOS native build**: arm64 binaries via CI
24. 📋 **NuGet publish**: Publish to NuGet.org
25. 📋 **CI/CD**: GitHub Actions workflows

## Risk register (and default mitigations)

* **Native load issues** → Use `runtimes/{rid}/native/`; add runtime diagnostics of probing paths; doc Gatekeeper steps on macOS.
* **Threading/UI blocking** → Background receive thread; marshal to UI safely; cancellation tokens; graceful Dispose.
* **Upstream drift** → Pin commit; add a "bump `ur_rtde`" CI job gated by tests.
* **DLL hell on net48** → Prefer `PackageReference`; optional fallback loader if probing fails.

## Communication & outputs (format you must return)

* Always return:

  * An updated **checklist** with current status.
  * **Blocking issues** (if any) + proposed resolutions.
  * **Next 1–3 concrete steps**.
* Keep responses concise, actionable, and free of speculation.
