# KAIST Mobility Challenge — CAV Control

This repository contains our team’s solution for the **KAIST Mobility Challenge**,
focusing on **connected and autonomous vehicle (CAV) control** in a multi-agent
ROS 2 simulation environment.

---

## Overview
- **Event**: KAIST Mobility Challenge  
- **Platform**: ROS 2–based multi-vehicle simulator  
- **Focus**: Trajectory tracking and motion control for CAVs  

---

## Simulator
This project is built on the official KAIST Mobility Challenge Simulator.

[![Simulator Repository](https://img.shields.io/badge/GitHub-Mobility_Challenge_Simulator-black?logo=github)](https://github.com/cislab-kaist/Mobility_Challenge_Simulator)

---

## What We Implement
- Vehicle state subscription and trajectory following
- Real-time acceleration and steering control
- Modular control logic for extensibility

---

## Architecture
- **ROS 2 Nodes**: state → control → command
- **Control**: reference trajectory tracking
- **Interface**: direct simulator integration

---

## Notes
- Structured Git workflow (feature → dev → main)
- Build artifacts and logs excluded via `.gitignore`
