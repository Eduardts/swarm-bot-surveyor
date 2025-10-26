# Swarm-Bot Site Surveyor

A simulation of autonomous drone swarms for construction site surveying using ROS and Gazebo.

## Project Structure

```
swarm-bot-surveyor/
├── gazebo_world/
│   └── terrain.world          # Gazebo world definition with terrain mesh
├── src/
│   ├── swarm_node.py          # ROS node for individual drone control and sensor data
│   ├── slam_processor.py      # ROS node for SLAM and point cloud generation
│   └── exporter.py            # Python script to export point cloud (LAS/OBJ)
├── launch/
│   └── survey_sim.launch      # ROS launch file to start simulation
├── README.md
└── requirements.txt
```

## Overview

Simulates autonomous drone swarms performing SLAM-based site surveys, generating 3D point clouds for topographic mapping.

## License

MIT License
