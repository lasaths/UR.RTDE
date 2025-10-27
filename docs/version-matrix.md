# Version Matrix

| Component | Version | Commit/Tag | Date |
|-----------|---------|------------|------|
| **ur_rtde** | v1.6.2 | [Latest](https://gitlab.com/sdurobotics/ur_rtde/-/tags/v1.6.2) | 2025 |
| **License** | MIT | - | - |

## Target Platforms

| Platform | Architecture | .NET Framework | Status |
|----------|--------------|----------------|--------|
| Windows | x64 | .NET 4.8 (Rhino 7) | 🎯 Target |
| Windows | x64 | .NET 8.0 (Rhino 8) | 🎯 Target |
| macOS | arm64 | .NET 8.0 (Rhino 8) | 🎯 Target |
| macOS | x64 (Rosetta) | .NET 8.0 (Rhino 8) | Optional |

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| Boost | ≥1.70 | ur_rtde dependency |
| CMake | ≥3.15 | Native build |
| .NET SDK | 8.0 | Managed build |

## UR Robot Compatibility

| Robot Series | PolyScope Version | Status |
|--------------|-------------------|--------|
| CB3/CB3.1 | ≥3.3 | ✅ Supported |
| e-Series | All | ✅ Supported |
| UR+ Series | ≥5.0 | ✅ Supported |

## RTDE Configuration

| Parameter | Default | Notes |
|-----------|---------|-------|
| Frequency | 500 Hz | e-Series/UR-Series |
| Frequency | 125 Hz | CB-Series |
| Port | 30004 | RTDE protocol |
| Protocol | TCP/IP | - |
