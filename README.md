## Instructions

We use the ROS middleware for the communication of our individual software modules, which automates both the process control and the inter-process communication. The developer can thus concentrate on implementing the functionality of the software. Your task should therefore also revolve around ROS, Linux and Docker.

1. Write a Dockerfile in which you install ROS2 Foxy. To do this, use Ubuntu 20.04 as the base image, since the ROS packages are already precompiled for this purpose. ( https://docs.ros.org/en/foxy/Installation.html )

2. Write a ROS node that writes the current CPU load as a relative value to a log file cpu_load.log every 5 seconds. You can implement the ROS node in C++ ( https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html ). To do this, expand your Dockerfile so that the node in the Dockerfile is started automatically.

3. Generate a docker-compose.yaml to automatically start and stop your Docker container with all necessary resources.

4. Write a shell script that reads the cpu_load.log file and outputs new messages in the file on the command line. The shell script should run outside of the Docker container.

5. Write a shell script that loads the CPU enough to test your pipeline.

As soon as you have finished the task , pack the Dockerfile, the ROS workspace and the scripts as an archive and send it to us. If you have any questions or something is not clearly formulated, just get in touch with us.