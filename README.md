# UR12e tensegrity workspace

ROS 2 Jazzy workspace copied from the current UR5e project, including its
uncommitted source changes. The original workspace remains untouched.

- `src/ur12e_motion`: renamed motion package, TRAC-IK launch configuration,
  and the existing 3D prism geometry, winding paths, and visualization.
- `src/dbar_tensegrity`: minimal independent 2D geometry package skeleton.
- `project_commands.txt`: build and UR12e simulation commands.
- `docs/legacy_ur5e_ikfast_setup_notes.txt`: historical notes only.

The simulation command explicitly selects `ur_type:=ur12e`; the custom
MoveIt launch also defaults to UR12e. Robot kinematics and meshes come from
the installed official UR description package. The IK client's position
limits match the installed UR12e configuration, including its +/- pi elbow
planning restriction.

The prism experiment retains its original feed point (20, 0, 20 inches)
and tool-to-prism offset (8 inches along tool Z). These describe the fixture,
not the robot model, and need to be confirmed for the new setup. They are
not D-bar parameters. Existing paths have not been validated for the UR12e.

Use a fresh terminal sourced with ROS Jazzy and this workspace's install
setup. Build outputs are regenerated locally; no old build/install/log
folders or Git history are copied. No hardware connection is configured.

See `project_commands.txt` to build and launch. Define the D-bar arrangement
and dimensions before implementing the new geometry.
