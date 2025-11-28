# Missile Trajectory Simulation & Optimization (3D)

A C++ simulation of a missile's flight path, incorporating aerodynamics, gravity, thrust, and wind effects. Features a 3D OpenGL visualization with a realistic missile model and explosion effects.

![Screenshot](https://via.placeholder.com/800x600?text=Missile+Simulation+3D)

## Features
- **Physics Engine**: RK4 numerical integration for accurate trajectory.
- **3D Visualization**: OpenGL rendering with camera controls (Pan, Zoom, Pitch).
- **Optimization**: Automatically calculates the optimal launch angle for max range.
- **Interactive**: User inputs for mass, thrust, burn time, and wind.

## Prerequisites
- **C++ Compiler**: MinGW (g++) or MSVC.
- **PowerShell**: For running setup scripts.

## Quick Start

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/yourusername/missile-sim-3d.git
    cd missile-sim-3d
    ```

2.  **Build and Run** (Automatic):
    ```powershell
    .\build.ps1
    ```
    *This script will automatically download necessary dependencies (GLFW) and compile the project.*

3.  **Run the Simulation**:
    ```powershell
    .\missile_sim_3d.exe
    ```

## Controls
- **W / S**: Pitch Camera
- **A / D**: Yaw Camera
- **Q / E**: Zoom
- **P**: Pause/Resume
- **R**: Restart
- **ESC**: Exit

## Project Structure
- `main.cpp`: Core source code (Physics + Visualization).
- `setup_dependencies.ps1`: Downloads and configures GLFW.
- `build.ps1`: Compiles the project.
