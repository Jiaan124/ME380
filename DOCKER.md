# ME380 Docker (ROS2 Humble + GUI)

## Build and run

```bash
docker compose build
docker compose up -d
docker compose exec me380 bash
```

Your repo is mounted at `/me380` inside the container. Build and run ROS2 from there:

```bash
cd /me380
colcon build --packages-select me380_driver
source install/setup.bash
# run your nodes, rviz2, etc.
```

## Seeing GUIs (RViz2, etc.)

### Linux

1. Allow the Docker daemon to talk to your X server:
   ```bash
   xhost +local:docker
   ```
2. Ensure `DISPLAY` is set (usually already is):
   ```bash
   echo $DISPLAY   # e.g. :0
   ```
3. Start the stack as above; GUI apps from the container will open on your desktop.

### macOS

Docker on Mac doesn’t share the host X11 socket, so you need XQuartz and TCP display:

1. Install [XQuartz](https://www.xquartz.org/) and start it.
2. In XQuartz: **Preferences → Security** → enable **“Allow connections from network clients”**. Restart XQuartz.
3. Run the container with:
   ```bash
   DISPLAY=host.docker.internal:0 docker compose up -d
   ```
   Or export it first:
   ```bash
   export DISPLAY=host.docker.internal:0
   docker compose up -d
   ```
4. Open a shell and run your GUI apps; windows will appear in XQuartz.
