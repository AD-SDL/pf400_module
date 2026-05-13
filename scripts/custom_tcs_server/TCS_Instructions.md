# Deploying Custom.gpl to the PF400 TCS Server

This document describes how to update the PF400 robot's TCS server with a new version of `Custom.gpl`.
`Custom.gpl` adds three custom kinematic commands to the TCS server: `JointToCart`, `CartToJoint`, and `RotateLoc`.

## Prerequisites

- Network access to `rplpf400.cels.anl.gov`
- `lftp` installed on your machine (`sudo apt install lftp`)
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

From the repo root:

```bash
lftp ftp://rplpf400.cels.anl.gov
set ftp:passive-mode off
cd /flash/projects/Tcp_cmd_server_pa
put path/to/Custom.gpl
quit
```

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

Expected output should include `RPL Custom Module 2.0`. If you see an older version number the project did not reload correctly — repeat step 3.

### 5. Test the commands

With the robot powered and attached, run a quick sanity check:

```
wherej
```

Take the 6 joint values from the response (strip the leading `0`) and run:

```
JointToCart <j1> <j2> <j3> <j4> <j5> <rail>
```

Compare the output against `wherec` — X, Y, Z, yaw, pitch, roll should match within rounding (~0.01mm).

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

| Command | Arguments | Returns | Description |
|---|---|---|---|
| `JointToCart` | `j1 j2 j3 j4 j5 rail` | `X Y Z yaw pitch roll` | Forward kinematics (FK) |
| `CartToJoint` | `X Y Z yaw pitch roll rail` | `j1 j2 j3 j4 j5 rail` | Inverse kinematics (IK) |
| `RotateLoc` | `j1 j2 j3 j4 j5 rail rotation_deg` | `j1 j2 j3 j4 j5 rail` | Rotate end effector yaw and return new joint angles |

All responses are prefixed with `0` on success (standard TCS status code) or a negative error code on failure.
