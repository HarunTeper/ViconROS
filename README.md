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

3. **Launching the Node:**  
    After configuring the network and DDS, launch the ViconROS node to start receiving and publishing Vicon data.

## Documentation

- [network-setup-tutorial.md](network-setup-tutorial.md): Step-by-step guide for network setup.
- `cyclone_dds_config.xml`: DDS configuration file—ensure the IP address matches your network setup.

## Support

For issues or questions, please open an issue in this repository.