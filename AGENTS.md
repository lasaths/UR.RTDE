# AGENTS.md — UR.RTDE Wrapper (Rhino 7 & 8, NuGet)

## Overview

You are an expert **software engineering agent** tasked with delivering a **C# wrapper** around the **SDU Robotics `ur_rtde` C++ library** that works in:

* **Rhino 7** (.NET Framework **4.8**, Windows x64)
* **Rhino 8** (.NET **8**) on **Windows x64** and **macOS (arm64)**

Deliver a **NuGet package** named **`UR.RTDE`** consumed by a **Grasshopper custom plugin**. Prioritize reliability, low latency, and clean developer ergonomics.

## Objectives (what to build)

1. A thin **native C ABI façade** over `ur_rtde` (opaque handles, arrays, int status codes; no exceptions crossing ABI).
2. A managed **C# wrapper** (namespace/assembly **`UR.RTDE`**) exposing:

   * **Control**: MoveJ, MoveL, SpeedJ/L, StopJ/L, SetTcp, SetPayload, watchdog.
   * **Receive**: ActualQ, ActualTcpPose, RobotMode, SafetyMode, RuntimeState, essential IO reads.
   * **IO**: basic digital IO where supported.
   * **Lifecycle**: Connect/Disconnect, timeouts, reconnect policy.
3. **Packaging** as a single **NuGet** with RID-specific native assets to "just work" in Rhino 7/8.
4. **Samples & docs** that prove end-to-end: connect → stream joints @ default rate → MoveJ → Stop.

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

## Acceptance criteria (must all pass)

* Loads and runs in **Rhino 7 (.NET 4.8)** and **Rhino 8 (.NET 8)** on listed platforms.
* Receives **ActualQ** continuously at default RTDE rate for **≥5 minutes** without UI blocking or drops.
* Executes **MoveJ** and **Stop** from the Grasshopper demo without exceptions.
* NuGet install into GH project requires **no manual native copying**.
* Clear, minimal docs enable a first-time Rhino user to succeed.

## Work plan (ordered checklist)

1. **Recon & pin**: Identify and pin the latest stable `ur_rtde` commit/tag; record license/NOTICE.
2. **Repo skeleton**: Create solution layout with `native/` (C façade), `src/UR.RTDE/` (managed), `samples/`, `docs/`, `ci/`.
3. **Native façade design**: Specify C ABI (CTRL/RECV handles; q[6], pose[6]; error codes). Review for cross-platform portability.
4. **Build `ur_rtde`** on Windows/macOS; then build the **C façade** linking against it (Release, x64/arm64).
5. **Managed P/Invoke**: Implement bindings, **IDisposable** ownership, error mapping, and a dedicated receive thread with events.
6. **Packaging**: Create NuGet with correct **`runtimes/`** layout; verify RID probing in both TFMs.
7. **Rhino/Grasshopper demo**: Simple GH components: Connect, StreamActualQ, MoveJ, Stop.
8. **Manual tests**: With URSim or robot: connect, stream @ default rate, MoveJ, Stop, disconnect/reconnect, simulate brief network hiccup.
9. **CI/CD**: Windows + macOS builds, unit tests (marshaling/lifecycle), artifact publish, NuGet pack.
10. **Docs**: Quickstart, troubleshooting tree, version matrix, safety notes.

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
