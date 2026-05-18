# URSim Docker (UR.RTDE)

Local **URSim e-Series** for `UR.RTDE`, Grasshopper, and integration tests.

Port layout matches [Multi-Actor-Interface-Library/docker/ursim](https://github.com/lasaths/Multi-Actor-Interface-Library/tree/main/docker/ursim).

## Quick start

From the **UR.RTDE** repository root:

```bash
docker compose -f docker/ursim/docker-compose.yml up -d
```

Open PolyScope: [http://127.0.0.1:6080/vnc.html?host=localhost&port=6080](http://127.0.0.1:6080/vnc.html?host=localhost&port=6080)

Wait until the robot is **powered on** and brakes are released in PolyScope (first boot can take **1–2 minutes**). `RTDEControl` will fail until the controller is ready even if `nc` on port 30003 succeeds.

Connect clients at **`127.0.0.1`** (macOS/Windows Docker Desktop).

Verify ports:

```bash
nc -zv 127.0.0.1 30003   # RTDE control (required for RTDEControl / MoveJ)
nc -zv 127.0.0.1 30004   # RTDE receive
```

## Published ports

| Port  | Use |
|-------|-----|
| 6080  | Browser VNC UI |
| 5900  | VNC client |
| 29999 | Dashboard |
| 30001 | Primary client |
| 30002 | URScript |
| 30003 | **RTDE control** |
| 30004 | **RTDE receive** |
| 50002 | External Control URCap (optional; see overlay below) |

## Optional: External Control URCap

Builds a custom image with the External Control JAR (helps `UploadScript` / port 50002 workflows):

```bash
docker compose -f docker/ursim/docker-compose.yml \
  -f docker/ursim/docker-compose.external-control.yml up -d --build
```

## Lifecycle

```bash
docker compose -f docker/ursim/docker-compose.yml logs -f
docker compose -f docker/ursim/docker-compose.yml stop
docker compose -f docker/ursim/docker-compose.yml down
```

Programs persist in the `ursim-programs` Docker volume.

## PolyScope

- Power on, release brakes, dismiss safety dialogs.
- **URSim in Docker**: Local Control is fine for Grasshopper (session tries several RTDE control flags).
- **Physical robot**: use your cell’s Remote Control requirements.
