# UR.RTDE <version> Release

Highlights
- Robotiq gripper support (URScript client + fast RTDE-register bridge)
- New RTDE register APIs (input/output) + custom URScript upload
- Updated docs, gated tests, and samples

Added
- UR.RTDE.RobotiqGripper (URScript) and UR.RTDE.URScriptClient
- UR.RTDE.RobotiqGripperRtde (fast RTDE-register bridge)
- Native facade: set_input_int/double/bit_register, get_output_int/double/bit_register, send_custom_script
- C# wrappers: register access on RTDEControl/RTDEReceive

Changed
- Documentation: README, Quickstart, Troubleshooting
- Tests gated by env: ENABLE_ROBOTIQ_TESTS, ENABLE_FT_TESTS

Notes
- Robotiq `rq_*` functions require the Robotiq URCap installed/enabled
- FT zeroing may be unavailable in URSim; keep ENABLE_FT_TESTS disabled

Install
```powershell
dotnet add package UR.RTDE -v <nuget-version>
```

Links
- NuGet: https://www.nuget.org/packages/UR.RTDE
- Repo: https://github.com/lasaths/UR.RTDE

