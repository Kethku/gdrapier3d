# gdrapier3d
Deterministic physics extension based on rapier3d for godot 4.0

## Build

Make sure you have rust installed via https://rustup.rs/

Once installed, build the project with `cargo build; cargo build --release`

## Installation

Create a .gdextension file that contains something like the
following:

```toml
[configuration]
entry_symbol = "gdext_rust_init"
compatibility_minimum = 4.1
reloadable = true

[libraries]
windows.debug.x86_64 = "res://{PATH TO THIS REPO}/target/debug/gdrapier3d.dll"
windows.release.x86_64 = "res://{PATH TO THIS REPO}/target/release/gdrapier3d.dll"
```

## High Level Overview

This extension is a relatively thin wrapper over the
https://rapier.rs/ physics engine and as such provides
deterministic physics. In its
current version, the system is designed to be ran via
`gdrollback` a sibling extension for rollback networking in
godot 4.0, but it wouldn't take much to enable it's use in
other contexts.

To work properly a R3DWorld node is registered as an
autoload. This node is responsible for managing bodies and
colliders in the scene.

R3DRigidBody nodes are automatically registered with the
R3DWorld node and will have their transform updated and
overwritten. R3D{Shape}Collider nodes added as children to a
R3DRigidBody serve as the collision shape.
