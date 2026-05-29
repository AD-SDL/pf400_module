# Deploying Custom.gpl to the PF400 TCS Server

This document describes how to update the PF400 robot's TCS server with a new version of `Custom.gpl`.
`Custom.gpl` adds custom commands to the TCS server for kinematics, force compliance, and height detection.

## Prerequisites

- Network access to `rplpf400.cels.anl.gov`
- `lftp` or FileZilla installed on your machine
- The robot's TCS server running on port 10100

## Steps

### 1. Back up the existing file (first time only)

```bash
lftp ftp://rplpf400.cels.anl.gov
cd /flash/projects/Tcp_cmd_server_pa
get Custom.gpl -o Custom.gpl.bak
quit
```

### 2. Upload the new Custom.gpl

**Via lftp:**

```bash
lftp ftp://rplpf400.cels.anl.gov
set ftp:passive-mode off
cd /flash/projects/Tcp_cmd_server_pa
put path/to/Custom.gpl
quit
```

**Via FileZilla:**

- Host: `rplpf400.cels.anl.gov`, Port: `21`, leave username and password blank
- Navigate to `/flash/projects/Tcp_cmd_server_pa` in the right panel
- Drag `Custom.gpl` from your local machine into that folder

### 3. Reload the project on the robot

- Open the robot web interface at `http://rplpf400.cels.anl.gov`
- Navigate to **Setup > Load/Compile Project**
- Select `Tcp_cmd_server_pa` and click **Load**
- Wait for the confirmation that the project compiled and started successfully

### 4. Verify the deployment

Connect via telnet and check that the new module version appears:

```bash
telnet rplpf400.cels.anl.gov 10100
```

Then send:

```
version
```

Expected output should include `RPL Custom Module 2.1`. If you see an older version number the project did not reload correctly — repeat step 3.

### 5. Test the commands

With the robot powered and attached, run a quick sanity check for each command group.

**Kinematics:**

```
wherej
```

Take the 6 joint values from the response (strip the leading `0`) and run:

```
JointToCart <j1> <j2> <j3> <j4> <j5> <rail>
```

Compare the output against `wherec` — X, Y, Z, yaw, pitch, roll should match within rounding (~0.01mm).

**Force Compliance** (requires XY Compliance license):

```
EnableCompliance 0
DisableCompliance
```

Both should return `0`. When compliance is enabled the horizontal arm axes should feel loose when pushed by hand.

**Height Detection** (requires Z Height Detection license):

Position the gripper at least 20mm above a surface, then:

```
HeightDetect 2 -400 -15
```

Should return `0 <detected_Z_height_mm>` rather than a license error.

## Rolling Back

If the new `Custom.gpl` causes issues, restore the backup and reload:

```bash
lftp ftp://rplpf400.cels.anl.gov
set ftp:passive-mode off
cd /flash/projects/Tcp_cmd_server_pa
put Custom.gpl.bak Custom.gpl
quit
```

Then reload the project following step 3 above.

## Custom Commands Reference

All responses are prefixed with `0` on success (standard TCS status code) or a negative error code and message on failure.

### Kinematics

| Command | Arguments | Returns | Description |
|---|---|---|---|
| `JointToCart` | `j1 j2 j3 j4 j5 rail` | `X Y Z yaw pitch roll` | Forward kinematics (FK): convert joint angles to Cartesian coordinates. Rail offset is added to X to match world coordinates. |
| `CartToJoint` | `X Y Z yaw pitch roll rail` | `j1 j2 j3 j4 j5 rail` | Inverse kinematics (IK): convert Cartesian coordinates to joint angles. Rail is subtracted from X before IK and returned unchanged. |
| `RotateLoc` | `j1 j2 j3 j4 j5 rail rotation_deg` | `j1 j2 j3 j4 j5 rail` | Rotate end effector yaw by rotation_deg degrees. Internally runs FK, applies rotation, then IK. Use to switch between narrow and wide plate orientations. |

### Force Compliance

Requires XY Compliance license installed on the controller.

| Command | Arguments | Returns | Description |
|---|---|---|---|
| `EnableCompliance` | `bias_torque_pct` | `0` | Enable horizontal force compliance. `bias_torque_pct` (0-100) sets a bias torque percentage to prevent axis drift. 0 = fully free, typical values 0-20. |
| `DisableCompliance` | none | `0` | Disable horizontal force compliance and return to normal position control. Always call after EnableCompliance. |

### Height Detection

Requires Z Height Detection license installed on the controller. Position the gripper at least 20mm above the surface before calling.

| Command | Arguments | Returns | Description |
|---|---|---|---|
| `HeightDetect` | `mode search_limit_mm max_force_n` | `detected_Z_mm` | Detect surface height. mode: 1=quick (0.5mm), 2=thorough (0.3mm). search_limit_mm: max downward travel (negative). max_force_n: contact force limit (negative). |

## License Requirements

| Feature | Required License |
|---|---|
| Kinematics (JointToCart, CartToJoint, RotateLoc) | GPL license (already installed) |
| Force Compliance (EnableCompliance, DisableCompliance) | XY Compliance |
| Height Detection (HeightDetect) | Z Height Detection |

To check installed licenses, navigate to **Utilities > Controller Options** in the robot web interface.
