# Flux3D Engine

[![Version](https://img.shields.io/badge/version-v1-blue.svg)]()
[![License](https://img.shields.io/badge/license-LGPL--2.1-orange.svg)](https://github.com/mncrzz/Flux3D/blob/main/LICENSE.txt)
[![Language](https://img.shields.io/badge/language-C%2B%2B17-green.svg)](https://web.archive.org/web/20171202203127/https://www.iso.org/standard/68564.html)
[![Status](https://img.shields.io/badge/status-Development-orange.svg)]()

A modern *3D engine* written in *C++*. Currently in **development**.

## 🎮 Features

- **Realistic Physics** — Integration of Bullet Physics for accurate simulation of physical objects
- **Dynamic Shadows** — Real-time shadow rendering using shadow mapping
- **Jelly Physics** — Deformable object system based on mass points and springs
- **Fully Interactive Scene** — Shoot objects, jump, and move freely through the environment
- **Skybox Rendering** — Cubic environment mapping for immersive backgrounds
- **Catppuccin Mocha UI Theme** — Beautiful and modern control interface
- **Real-time Lighting** — Dynamic positioning and control of light sources
- **Wireframe Mode** — Debug visualization of mesh structure

## 🛠️ Technology Stack

- **C++17** — Popular C++ standard,
- **OpenGL 3.3** — 3D graphics API for rendering,
- **GLFW** — Window management and input handling,
- **GLM** — OpenGL Math Library,
- **Bullet Physics** — 3D physics engine,
- **ImGui** — Immediate mode graphical user interface,
- **stb_image** — Image loading for textures.

## 📋 Requirements

- **CMake 3.10** or higher
- **C++ compiler** with C++17 support
- **OpenGL 3.3** or higher

## 🚀 Installation & Compilation

### Linux / macOS

```bash
# Clone the repository
git clone https://github.com/mncrzz/Flux3D.git
cd Flux3D

# Create build directory
mkdir build && cd build

# Generate CMake project
cmake ..

# Compile
make -j$(nproc)

# Run the application
./Flux3D
```

### Windows

```cmd

git clone https://github.com/mncrzz/Flux3D.git
cd Flux3D
mkdir build && cd build
cmake ..
cmake --build . --config Debug

```

> [!IMPORTANT]  
> If you are building a project on Windows, do not change `cmake --build . --config Debug` to `cmake --build . --config Release`, otherwise an error will occur.

## 👀 Preview

![Flux3D Logo](logo.jpg)

## 🪱 Issues and feedback

Find an **issue**? [Report about it!](https://github.com/mncrzz/Flux3D/issues)

---

Made with ❤️ by [mncrzz](https://github.com/mncrzz/) and [contributors](https://github.com/mncrzz/Flux3D/graphs/contributors)
