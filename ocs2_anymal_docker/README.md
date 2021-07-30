# Using OCS2 Docker

These instructions do not assume that ROS or catkin exist on the 
host system. Therefore adapt the following instructions as needed
on your machine.

## Intro

Here's an outline of the steps needed for setting up a docker container for OCS2 development:  
1. Install base dependencies on the host machine (separate instructions are provided for platforms using NVIDIA GPU + CUDA).  
2. Building an image.  
3. Launching a container.  
4. Creating and using a Catkin workspace.  

There is also the option to pull pre-built images directly from our internal Docker 
hosting service [Harbor](https://registry.leggedrobotics.com/). Instructions for
using harbor are presented [below](##5.Using Harbor).  

## 1. Dependencies

The `bin/install.sh` script has been provided as easy means for installing docker. 

For CPU-only builds, simply run:
```commandline
cd /path/to/package/ocs2_anymal_docker/bin
./install.sh
```

For hosts with an NVIDIA GPU, we can build a NVIDIA+CUDA enabled container using:
```commandline
cd /path/to/package/ocs2_anymal_docker/bin
./install.sh --nvidia
```

If errors occur, please read the contents of `install.sh` and modify the script as necessary.

Once the installation finishes, reboot the computer so the docker service and groups are running.

## 2. Building an image

For building CPU-only images:
```commandline
cd /path/to/package/ocs2_anymal_docker/bin
./build.sh
```

The script defaults to CPU-only builds, but if for some reason cpu needs to be specified explicitly:
```commandline
cd /path/to/package/ocs2_anymal_docker/bin
./build.sh --platform=cpu
```

Respectively, for NVIDIA GPU platforms:
```commandline
cd /path/to/package/ocs2_anymal_docker/bin
./build.sh --platform=gpu
```

The default name of the generated image will be something like `ocs2/ocs2-anymal:20.03-gpu`. The 
tag `:20.03-gpu` indicates that this has been built for supporting release version `20.03` of the 
`anymal-research` codebase and supports NVIDIA platforms. 

The release version of `anymal-research` can be specified using the `--release=XX.YY` argument:
```commandline
./build.sh --release=20.03
```

The **name and tag** of the image can be specified explicitly using the `--image=<MY_CUSTOM_NAME>`:
```commandline
./build.sh --image=ocs2/ocs2-custom-dev:latest
```

Once a build starts, it would be a good time to get a coffee, it's gonna take a while.

## 3. Launching a container.

The `run.sh` script has been provided to launch an OCS2 container. This script uses arguments identical 
to the `build.sh` script. So if we just built GPU image, we can launch a container from this image using:
```commandline
./run.sh --platform=gpu
```

You'll know it's working if you see this output:
```commandline
user@host:~/git/ocs2_anymal/ocs2_anymal_docker/bin$ ./run.sh --platform=gpu

  ____   _____  _____ ___      _))
 / __ \ / ____|/ ____|__ \   >  *\     _~
| |  | | |    | (___    ) |   `;'\\__-' \_
| |  | | |     \___ \  / /       | )  _ \ \
| |__| | |____ ____) |/ /_      / /    w w
 \____/ \_____|_____/|____|    w w

 OCS2: Optimal Control for Switched Systems

[OCS2-Docker::Entrypoint] Launching shell as user 'user'
user@host:~$
```

**IMPORTANT** to keep in mind:  
1. The current docker image has been built to launch containers which login to the host user.  
2. All logins configure the environment by sourcing the copy of `src/bashrc` in the container.  
3. The host user's home directory is mounted in the container. This allows us to maintain the 
catkin workspaces in our home directories, and store all persistent data locally.  
4. Any changes made to the containers file system (except for the mounted home directory) are not
persistent and will be lost after exiting the container.  

## 4. Creating and using a Catkin workspace

Now that the container is launched, you can create/setup a catkin workspace within which you can build 
packages using the environment and dependencies provided by the image.

## 5. Using Harbor

First, login credentials are needed in order to access the registry. These can be acquired
by contacting Tom ([tom.lankhorst@mavt.ethz.ch](tom.lankhorst@mavt.ethz.ch)). Once you have
a `USERNAME` and `PASSWORD`, you can use the docker front-end to perform the login from terminal:
```commandline
docker login --username USERNAME registry.leggedrobotics.com
```

Once access has been set up, it is possible to pull from the registry using:
```commandline
docker pull registry.leggedrobotics.com/IMAGE
```

Where `IMAGE` is the name of the target image to be pulled. The name must exist in the 
registry otherwise the command will fail.

Example:
```commandline
docker pull registry.leggedrobotics.com/ocs2/ocs2-anymal:21.03-gpu
```

----
