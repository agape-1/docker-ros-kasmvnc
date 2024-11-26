## Simulation

### Requirements

* Docker

### Getting started

Clone the repository and run the following command:


```bash
docker compose up gz_sim
```

#### Nvidia GPUs
If you have Docker Nvidia support, run this command instead:

```bash
docker compose up gz_sim_nvidia
```

Visit http://loclahost:3000 or `http://localhost:$VNC_PORT` to access the Gazebo GUI.

This simulation also supports Gazebo's [visualization](https://app.gazebosim.org/visualization) tool. Access the tool and enter ws://localhost:9002 or `ws://localhost:$WEBSOCKET_PORT` and press 'Connect'.

### Quick tips

* If you accidentally closed the application and ended up with a blank screen, simply right click the background and you'll see an application menu to reopen Gazebo.

* Use a Chromium based browser for seamless clipboard integration.

* By default, the docker container exposes the network across `0.0.0.0`, which allows other clients to access your simulation from their browser. To add authentication, supply a `CUSTOM_USER` and `CUSTOM_PASSWORD` variables to `.env` and update the container from the [configuration](#configuration) section.

* Additional VNC [options](https://github.com/linuxserver/docker-baseimage-kasmvnc?tab=readme-ov-file#options) are available.

### Configuration

Environment variables are supplied in `.env` for quick and intuitive flexibility. Simply change the variables and run the following command:

```
docker compose up gz_sim --force-recreate --build
```

to rerun the container with the updated configuration. Adjust as necessary similar to [nvidia startup steps](#nvidia-gpus).

#### .env Table


[//]: # (Table was generated by ChatGPT and then modified/double checked for its contents)

| Variable Name              | Default Value               | Description                                                              |
|----------------------------|-----------------------------|--------------------------------------------------------------------------|
| `ROS_DISTRO`               | `humble`                   | Specifies the ROS (Robot Operating System) distribution to use.          |
| `GZ_VERSION`               | `garden`                   | Defines the version of Gazebo to use.                                    |
| `WEBOSCKET_GZLAUNCH_FILE`  | `websocket.gzlaunch`       | Specifies the Gazebo launch file for WebSocket configuration. Unless you need to configure gz launch, this can be safely ignored.           |
| `GZ_SIM_OPTIONS`           | (empty)                    | Options for customizing Gazebo simulator options at runtime, identical to `gz sim $GZ_SIM_OPTIONS`. Empty by default.     |
| `WEBSOCKET_PORT`           | `9002`                     | Port number used for Gazebo's [visualization](https://app.gazebosim.org/visualization) tool.                               |
| `VNC_PORT`                 | `3000`                     | Port number used to access the Gazebo GUI via the browser.         |

### Development

As a result of using [`KasmVNC`](https://github.com/linuxserver/docker-baseimage-kasmvnc), changes to the Docker container will require a full removal of the container before rebuilding.


```bash
docker stop $DOCKER_ID && docker rm $DOCKER_ID

docker compose up gz_sim --force-recreate --build

```

where `$DOCKER_ID` is the previously composed Docker container ID, if any.