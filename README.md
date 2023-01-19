## Assignment Instructions

We use the ROS middleware for the communication of our individual software modules, which automates both the process control and the inter-process communication. The developer can thus concentrate on implementing the functionality of the software. Your task should therefore also revolve around ROS, Linux and Docker.

1. Write a Dockerfile in which you install ROS2 Foxy. To do this, use Ubuntu 20.04 as the base image, since the ROS packages are already precompiled for this purpose. ( https://docs.ros.org/en/foxy/Installation.html )

2. Write a ROS node that writes the current CPU load as a relative value to a log file cpu_load.log every 5 seconds. You can implement the ROS node in C++ ( https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html ). To do this, expand your Dockerfile so that the node in the Dockerfile is started automatically.

3. Generate a docker-compose.yaml to automatically start and stop your Docker container with all necessary resources.

4. Write a shell script that reads the cpu_load.log file and outputs new messages in the file on the command line. The shell script should run outside of the Docker container.

5. Write a shell script that loads the CPU enough to test your pipeline.

As soon as you have finished the task , pack the Dockerfile, the ROS workspace and the scripts as an archive and send it to us. If you have any questions or something is not clearly formulated, just get in touch with us.
## Directory structure
All the run scripts can be found in the root directory.
```bash

├── build.sh
├── cpu_load.log
├── docker-compose.yml
├── Dockerfile
├── log
├── read_log.sh
├── README.md
├── ros_entrypoint.sh
├── run.sh
└── src
```
`src` has the code for the ROS Node.
```bash
src
└── cpp_pubsub
    ├── CMakeLists.txt
    ├── include
    │   └── cpp_pubsub
    ├── package.xml
    └── src
        ├── publisher_member_function.cpp
        └── subscriber_member_function.cpp
```
Important: FOr the purpose of this assignment only a publisher node was required hence the subscriber node isn't thoroughly tested.

## Dependencies
```bash
Docker [version 20.10.22, build 3a2c30b]
Docker Compose [version v2.14.1]
stress [1.0.5]
C++ 17
```
## How to run?
1. In the root directory, simply run 
```
docker compose up
```
2. Then open another terminal and run 
```
watch -n 10 ./read_log.sh 
```
 to read and publish the CPU utilization as it changes. `-n 10` is time seconds at which it will refresh the terminal. `read_log.sh` is the script that shows updates in the `cpu_load.log` file
3. Open another terminal and then use this and you'll see a spike in your cpu utilization
```bash
stress --cpu 2 --timeout 60
```
4. Once you're done, don't forget to use:
```bash
docker compose down
```
to remove stopped containers

## FAQ
1. How to build and run the docker image without `docker compose`?

    Use the scripts `build.sh` and `run.sh` in the root directory. `run.sh` will run the docker interactively

2. How to build image from scrath without using `cache` with docker compose?

    `docker compose build --no-cache`

