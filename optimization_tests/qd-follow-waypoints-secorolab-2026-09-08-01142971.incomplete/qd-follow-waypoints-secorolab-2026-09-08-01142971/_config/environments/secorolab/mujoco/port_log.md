# secorolab scene — port log

**Source**: `environments/secorolab/3d-mesh/secorolab.stl` **and** `environments/secorolab/json-ld/floorplan.fpm.json`
(+ `environments/secorolab/maps/secorolab.pgm` / `.yaml`, unmodified — already a standard ROS
`map_server` map, co-registered with the mesh by the originating `secorolab/scenery_builder` tool).
All three are derived by scenery_builder from the same `secorolab.fpm` floorplan, so they agree by
construction. The Gazebo half uses only the STL, via `files/world.sdf.xacro`, which loads it as one
static collision+visual mesh with no further structure.

**Two sources, because MuJoCo needs both.** The STL is a *fused surface* mesh: a wall with a door cut
out of it is a single non-convex plate. MuJoCo collides a mesh by its **convex hull**, and the hull of
that plate is the solid wall. The json-ld is the mesh's own structured source — every wall and column
as an exact convex polyhedron, with `Entryway` nodes naming the walls they void — so it can be
collided directly, with doorways as the *absence* of geometry rather than holes in it.

**Pipeline**:

```
environments/secorolab/3d-mesh/secorolab.stl  (+ ../json-ld/*.json, found beside it)
  --[ roqsim scenes jsonld-to-scene ]--> mujoco/scene.json + mujoco/meshes/*.obj
                                      1 visual mesh (collide: false, the STL)
                                      45 convex wall/column colliders (render: false, from the json-ld)
  --[ roqsim scenes scene-to-mjcf ]-->   mujoco/secorolab.xml
```

**Verified against** the map, numerically, because the check that matters here cannot be seen:

| check | result |
| --- | --- |
| sphere probe (r = 0.175 m, TB4) at each of the 8 `Entryway` centres the json-ld declares | 0.23 m clearance each side (a ~0.81 m opening); the main entry 0.275 m |
| same probe mid-wall (4 points) and at the corridor centre | walls solid (−0.225 m), corridor 0.675 m — a 1.7 m corridor |
| flood fill from (−1.62, −3.54), robot radius eroded, world vs. `maps/secorolab.pgm` | **95.4 %** of the map's 124.7 m² navigable area reachable, in **one** connected component spanning x ∈ [−5.77, 5.78], y ∈ [−6.28, 6.52] |
| lidar, 360 rays at 0° and at 15° down tilt, corridor centre and room 254 | 0 floor returns at both |

The residual 4.6 % is the eroded rind along walls: the colliders are exact 0.10 m boxes while the map
is a 5 cm raster of the same walls. No pocket is cut off — the whole building is one component.

**A render cannot verify this scene, and an earlier version of this log claimed it could.** See "The
wrong route" below: the first port was checked by rendering it top-down and comparing against
`maps/secorolab.jpg` by eye, and it passed, because the triangles were always right. Only physics was
wrong. Any future change here must be re-checked with the numbers above, not with a picture.

**Floor, and why it is drawn**: the walls are the only geometry either source produces, so
`scene-to-mjcf`'s own ground plane is the floor. It bakes as **two** geoms — `floor`, the collider at
`ground_z` (`group="3" rgba="0 0 0 0"`, what the robot stands on and what `contact_monitor:
{ignore: [floor]}` names), and `floor_visual` 2 mm lower, non-colliding and opaque, which is what every
renderer shows. Two, because a single drawn plane at `ground_z` z-fights with any floor a scene brings
of its own; secorolab has none, so here the plane is the floor.

An earlier version of this log said the plane must stay invisible, because drawn it becomes a phantom
lidar ring that closes doorways in nav2's costmap — measured at 167 of 360 rays. **That measurement
used a 15° down tilt, and this robot does not tilt.** Over the whole of
`doorway-rebake-pilot-2026-08-13-12462389` the TB4's pitch never exceeded **0.78°** (mean 0.62°, close
to its resting attitude on the caster). At the rplidar's 0.192915 m height:

| pitch | nearest floor return | returns of 360 |
| --- | --- | --- |
| 0.00° | — | 0 |
| **0.78° — worst observed** | past the plane's extent | **0** |
| 1.25° | ~8.9 m (the plane's diagonal reach) | first appear |
| 2.00° | 5.6 m | 42 |
| 15.00° — the old probe | 0.75 m | 360 |

And the doorways were never closed by the floor: they were closed by the **convex hull** of the wall
plates, which is what `floor-revert-check-2026-08-13-10172692` was really observing. A scanner should
see the ground, and at the pitch this robot reaches it sees none of it.

The mechanism that log documented is correct and worth keeping: a material overrides the geom's `rgba`,
and `mj_ray`/`mj_multiRay` skip a geom exactly when its *resolved* alpha is 0 — independently of any
group mask. That is what made the alpha-0 plane invisible to the lidar, and it is why the collider half
can stay hidden while the visual half is seen. The margin over the observed pitch is not large, so it
is pinned by `roqsim_scenes/tests/test_ground_plane_lidar.py` rather than left to be re-argued.

**Capability class**: direct — no new substrate plugin. `roqsim scenes jsonld-to-scene` was added for
this port, but it only wires together `roqsim.floorplan_collision.wall_colliders` (which already existed,
for the `floorplan` run-time plugin) and the shared `scene-to-mjcf` bake.

**Effort**: two CLI runs and the verification script. The verification is the part worth budgeting
for — the port itself is minutes.

---

## The wrong route, and what it cost

Recorded because the mistake was cheap to make, invisible once made, and is still one command away.

The first port ran `roqsim scenes sdf-to-scene --collision-only` over `world.sdf.xacro` — i.e. it baked
the **STL** and never opened the json-ld. What followed:

1. `scene-to-mjcf` refused with *"coplanar vertices, cannot compute convex hull"* on the first of 88
   zero-thickness wall quads. **That error was the signal to change route, and was read as a substrate
   quirk to work around.** A one-off script (never committed, so the world could not be regenerated
   from the two commands this log named) replaced each quad with a 0.05 m box.
2. The remaining 13 parts were non-convex plates with the door cutouts in them. Nothing refused those:
   `sdf_to_scene`'s `_check_hulls_are_faithful` guard judges only *watertight* parts and only on a
   hull/mesh volume ratio over 4.0 — these are open STL skins scoring 1.5–2.8, because a 0.8 × 2.0 m
   doorway is a rounding error in a 13 m wall's volume and 100 % of its passability.
3. So all 8 doorways were baked shut. **12.9 %** of the building was reachable — one room — while
   nav2 planned on a map where 100 % is connected.

It survived four campaigns. Campaign `nav2-baseline-navigator-2026-08-13-00405348` measured it
directly ("in 25 of 25 runs the robot never got west of x = 1.11, stalling at the doorway near
(1.11, 3.70)") and it was diagnosed as nav2's 1.70 m inflation against a 0.90 m map passage — a real
constraint, but not the one holding the robot: the doorway was walled. `floor-revert-check-2026-08-13-10172692`
then sat pressed against the invisible slab at x = −1.11 for 66 s of an 85 s run, `follow_waypoints`
returned ABORTED, and the run still reported `passed`.

Two things now catch it: `sdf-to-scene` runs a scene-level passage test that refuses when the hulls
disconnect free space the triangles leave connected, and the `scene-porting` skill maps the
coplanar-hull error to the route change rather than to a workaround.
