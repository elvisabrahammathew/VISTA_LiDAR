# Monash VISTA LiDAR Team Repo 

A LiDAR-powered smart infrastructure platform developed as part of the Monash VISTA program.

## Run the C++ program without building

Ready-to-run Release packages are in [`publish/`](publish/README.md): Windows
x64, Linux x86-64 and Linux ARM64. Review the DeviceConfig.txt beside your chosen
executable before starting. Release builds refresh that platform's package
automatically; private Grafana tokens and local runtime data are not committed.

## Build the C++ application

See [the teammate build guide](embedded_C/docs/BUILDING.md) for Visual Studio
2019/2022, VS Code, bundled C++ libraries, Linux builds, and the pre-commit checklist.
In VS Code, open `VISTA-CPP.code-workspace` for the full repository, or open
`embedded_C` directly when using that folder alone. In Visual Studio, open the
`embedded_C` folder (CMake project), not a generated solution from another PC.

## Project Overview

This project is being developed as part of the Monash VISTA Program.

The objective is to build a scalable perception system capable of understanding how people interact with physical spaces using LiDAR sensors.

The system converts raw point cloud data into meaningful operational insights including but not limited to:

- Real-time pedestrian detection
- Multi-object tracking
- Occupancy estimation
- Crowd density analysis
- Pedestrian flow visualisation
- Congestion detection
- Historical analytics
- Digital twin visualisation

Rather than simply counting people, the platform aims to provide spatial intelligence that can support smarter campus operations, research, and future smart-city applications.
