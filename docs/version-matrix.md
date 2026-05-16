# Version Matrix

| Component | Version | Commit/Tag | Date |
|-----------|---------|------------|------|
| **ur_rtde** | 1.6.3 (pinned commit) | [68ac4e1](https://gitlab.com/sdurobotics/ur_rtde/-/commit/68ac4e18f357f8e9361bfc5eef344acd9aa241be) | 2026-05-16 |
| **License** | MIT | - | - |

## Target Platforms

| Platform | Architecture | .NET Framework | Status |
|----------|--------------|----------------|--------|
| Windows | x64 | .NET 4.8 (Rhino 7) | Target Target |
| Windows | x64 | .NET 8.0 (Rhino 8) | Target Target |
| macOS | arm64 | .NET 8.0 (Rhino 8) | Target Target |
| macOS | x64 (Rosetta) | .NET 8.0 (Rhino 8) | Optional |

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| Boost | >=1.70 | ur_rtde dependency |
| CMake | >=3.15 | Native build |
| .NET SDK | 8.0 | Managed build |

## UR Robot Compatibility

| Robot Series | PolyScope Version | Status |
|--------------|-------------------|--------|
| CB3/CB3.1 | >=3.3 | [OK] Supported |
| e-Series | All | [OK] Supported |
| UR+ Series | >=5.0 | [OK] Supported |

## RTDE Configuration

| Parameter | Default | Notes |
|-----------|---------|-------|
| Frequency | 500 Hz | e-Series/UR-Series |
| Frequency | 125 Hz | CB-Series |
| Port | 30004 | RTDE protocol |
| Protocol | TCP/IP | - |
