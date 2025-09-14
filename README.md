# Robot Communication System with Time-Sync (HybridSynchronizer)

A high-performance ROS1-based robot-to-server communication system implementing temporal synchronization and intelligent bandwidth management for multi-sensor fusion scenarios like UrbanLoco dataset.

## Table of Contents

- [Project Overview](#project-overview)
- [Architecture](#architecture)
- [Why HybridSynchronizer?](#-why-hybridsynchronizer)
- [Bandwidth Management Approach](#-bandwidth-management-approach)
- [Technical Implementation](#-technical-implementation)
- [Test Results](#-test-results)
- [Docker Deployment with Static Network](#-docker-deployment-with-static-network)
- [Performance Metrics](#-performance-metrics)

---

## Project Overview

### Key Features

- **Temporal synchronization** using `message_filters::ApproximateTime`
- **Intelligent bandwidth management**
- **Priority-based sensor dropping** during network congestion
- **Multi-sensor fusion** (LiDAR + IMU + GNSS + Cameras)
- **YAML-configurable** parameters for production flexibility
---

## Architecture

### System Overview
