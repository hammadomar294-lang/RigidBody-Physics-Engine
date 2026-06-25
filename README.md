# 2D Physics Engine

A personal long-term project focused on building a modular 2D physics engine entirely from scratch in **C++**.

The purpose of this project is to understand the mathematics, simulation techniques, engine architecture, and software engineering principles behind modern physics engines rather than relying on existing physics libraries.

---

## Screenshots

https://github.com/user-attachments/assets/cf1b98f9-8d24-4686-a7bf-73cbe8eb9e40

---

> **This project was built completely from scratch.**
> No external physics engine (such as Box2D or Chipmunk2D) is used.

---

## Features

### Physics

- Circle vs Circle Collision Detection
- Circle vs Box Collision Detection
- Box vs Box Collision Detection
- Impulse-based Collision Resolution
- Position Correction
- Linear & Angular Velocity
- Rotation
- Gravity
- Friction
- Sleeping System

### Constraints

- Distance Joints
- Springs
- Rope Constraints
- Pendulum Simulation

### Demonstrations

- Projectile Motion
- Spring Motion
- Rope Simulation
- Pendulum Simulation
- Tower Destruction
- Mouse Drag Interaction
- Physics-based Projectile Shooting

### Optimization

- Broad Phase Spatial Grid
- Narrow Phase Collision Detection

---

## Controls

| Input | Action |
|-------|--------|
| Left Mouse | Drag bodies |
| Right Mouse | Spawn random body |
| Shoot Mode | Launch projectile |
| Mouse | Aim projectile |

---

## Project Structure

```
math/
    Vector mathematics
    Utility math functions

physics/
    Bodies
    Collision Detection
    Collision Resolution
    Joints
    Springs
    Sleeping
    Broad Phase

UI/
    RayGui (future)

tester.cpp
    Sandbox application
```

---

## Dependencies

- C++17
- Raylib
- RayGui *(planned for future sandbox UI)*

---

## Build

Compile:

```bash
make app
```

Run:

```bash
make run
```

Clean:

```bash
make clean
```

---

## Philosophy

The objective of this project is not simply to recreate an existing physics engine, but to learn how one is designed and implemented from first principles.

Each major system has been implemented incrementally to understand the underlying algorithms and engineering decisions behind modern game engines.

---

## Roadmap

### ✅ Phase 1 — Physics Engine

Core physics simulation.

Completed:

- Rigid Bodies
- Collision Detection
- Collision Resolution
- Rotation
- Friction
- Sleeping
- Springs
- Ropes
- Pendulums
- Physics Demonstrations

---

### 🔄 Phase 2 — Advanced Physics

Planned features:

- Revolute Joints
- Motors
- Vehicle Physics
- Soft Constraints
- Ragdolls
- Continuous Collision Detection (CCD)

---

### 🎮 Phase 3 — Games

Build complete games powered entirely by the engine.

Potential projects:

- Angry Birds Clone
- Physics Puzzle Game
- Bridge Builder

## Repository Status

**Version:** v1.0

**Status:** Phase 1 Complete

This repository will continue evolving as new engine features are implemented in future phases.
