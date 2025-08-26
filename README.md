# ViconROS

This repository provides ROS2 integration for Vicon motion capture systems, enabling the collection and distribution of Vicon data over a network. It allows users to receive real-time motion capture data from a Vicon system and publish it as ROS2 topics for use in robotics applications.

## Features

- Interfaces with Vicon motion capture systems.
- Publishes Vicon data as ROS2 messages.
- Supports networked data transmission for distributed systems.

## Getting Started

1. **Network Configuration:**  
    Ensure your network is set up correctly for ROS2 communication. Refer to the [network-setup-tutorial.md](network-setup-tutorial.md) for detailed instructions.

2. **Cyclone DDS Configuration:**  
    Edit the `cyclone_dds_config.xml` file to set the correct IP address for your network interface. This is essential for proper data transmission between nodes.

3. **Download the Vicon DataStream SDK:**  
    Visit the [Vicon DataStream SDK download page](https://www.vicon.com/software/datastream-sdk/) and download the latest version for Linux.  
    Extract the archive and move the `Linux64` folder into your workspace directory at `/home/ubuntu/workspace`.

## Running the ViconROS Node

You can build and launch the ViconROS node using VS Code tasks for a streamlined workflow.

### 1. Build the Workspace

Use the `ROS2Build` task to build all packages in your workspace:

1. Open the Command Palette (`Ctrl+Shift+P`).
2. Select **Tasks: Run Task**.
3. Choose **ROS2Build**.

This will compile all ROS2 packages, including ViconROS.

### 2. Launch the Vicon Node

After building, use the `ROS2Launch` task to start the Vicon node:

1. Open the Command Palette (`Ctrl+Shift+P`).
2. Select **Tasks: Run Task**.
3. Choose **ROS2Launch**.

This will launch the ViconROS node and begin publishing motion capture data as ROS2 topics.

## Documentation

- [network-setup-tutorial.md](network-setup-tutorial.md): Step-by-step guide for network setup.
- `cyclone_dds_config.xml`: DDS configuration file—ensure the IP address matches your network setup.

## Support

For issues or questions, please open an issue in this repository.