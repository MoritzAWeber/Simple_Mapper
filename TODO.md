# TODO

This file tracks concrete work that is not implemented in the repository.

## Tooling and documentation

- [ ] Add a reusable RViz 2 configuration if one is needed for the demo.

## Mapping and SLAM

- [ ] Mark observed free space along each laser ray instead of recording only
  occupied endpoints.
- [ ] Estimate robot motion from sensor observations, for example through scan
  matching, instead of relying exclusively on exact simulated odometry.
- [ ] Introduce a `map` frame and publish the appropriate `map -> odom`
  correction when pose correction is implemented.
- [ ] Add loop-closure detection and pose-graph optimization if the project is
  intended to become a complete SLAM system.

## Simulation

- [ ] Add configurable noise or uncertainty to robot motion and odometry.
- [ ] Add a configurable laser model with measurement noise and missed returns.
- [ ] Replace or extend the fixed four-wall room with a configurable environment.

## Tests

- [ ] Add functional tests for the circular odometry trajectory and published
  TF frame relationships.
- [ ] Add tests for ray-to-wall intersection distances and laser scan metadata.
- [ ] Add tests for world-to-grid conversion, grid bounds, occupied-cell updates,
  and occupancy-grid metadata.
- [ ] Add a launch test that verifies all three demo nodes and their ROS
  interfaces start successfully.
