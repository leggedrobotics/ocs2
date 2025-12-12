## OCS2 Jazzy Docker

Build:

```bash
docker build -f docker/Dockerfile.jazzy -t ocs2:jazzy .
```

Run:

```bash
docker run --rm -it --net=host ocs2:jazzy
```

Note: the Dockerfile clones `ocs2_robotic_assets` from its `ros2` branch.
