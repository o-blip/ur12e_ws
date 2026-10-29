# UR12e D-bar tensegrity workspace

ROS 2 Jazzy workspace for developing winding motion for a planar D-bar structure.

- `src/ur12e_motion`: MoveIt planning and execution client and TRAC-IK launch configuration.
- `src/dbar_tensegrity`: independent planar D-bar geometry definitions.
- `project_commands.txt`: build and UR12e simulation commands.
- `docs/legacy_ur5e_ikfast_setup_notes.txt`: historical notes only.

The simulation command explicitly selects `ur_type:=ur12e`; the custom
MoveIt launch also defaults to UR12e. Robot kinematics and meshes come from
the installed official UR description package.

The IK client waits for Enter, plans a joint-space approach to node 1, and
executes three Cartesian segments: wind 1->3, transfer directly 3->2, then
wind 2->4. After each wind segment it waits for Enter so the string can be
secured and cut. Each segment is planned from the current robot state;
incomplete paths or failed executions stop the sequence. Collision checking
is enabled against the MoveIt planning scene (the physical fixture is not
currently modeled there).

Node 1 is at (0.30, 0.30, 0.0) meters in base_link, with the D-bar in the XY
plane and the moving feed 0.01 meters above the nodes. Tool orientation is
180 degrees about base X (tool +Z down) and the feed-to-tool offset is
provisionally zero. The transfer stays
at the same feed height. Joint velocity and acceleration scaling default to
0.1 and can be set using `velocity_scaling` and `acceleration_scaling` ROS
parameters. Approach motion is joint-space planned, not a straight feed path.

Launch the simulation before running `ros2 run ur12e_motion ik_client`.
The client now executes motion after the initial Enter prompt. These poses
and the provisional orientation still need validation in simulation.

The previous prism motion and visualization were removed.

Use a fresh terminal sourced with ROS Jazzy and this workspace's install
setup. No hardware connection is configured.

See `project_commands.txt` to build and launch.
