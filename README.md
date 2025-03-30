## Setup

Run Docker Desktop and proceed through the initial installation, then open a terminal, navigate to this repository (e.g. `cd ~/environment`, change depending on where you clone it), and run the following commands:

```sh
cd environment
cd ros2

# if using windows (make sure you run these commands from a WSL shell):
cd windows
# else if using mac:
cd mac
docker compose up -d

# To attach to the container call this in every terminal you want to open:
docker exec -it spot-sim bash

# To close the container:
docker compose down;

# To reopen:
docker compose up -d;
```
The container should look this:
```sh
# ROS 2:
hack@spot-sim:~$
```
