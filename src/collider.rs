mod collider_macros;

use godot::{
    engine::{BoxMesh, CapsuleMesh, CylinderMesh, Engine, Mesh, MeshInstance3D, SphereMesh},
    prelude::*,
};
use rapier3d::prelude::*;

use crate::{collider, rigid_body::R3DRigidBody};

pub use self::collider_macros::R3DCollider;

collider!(R3DBallCollider, radius: f32 = 0.5);
impl R3DCollider for R3DBallCollider {
    fn get_shape(&self) -> SharedShape {
        SharedShape::ball(self.radius)
    }

    fn get_debug_mesh(&self) -> Gd<Mesh> {
        let mut mesh = SphereMesh::new_gd();
        mesh.set_radius(self.radius);
        mesh.set_height(self.radius * 2.0);
        mesh.upcast()
    }
}

collider!(
    R3DCapsuleCollider,
    radius: f32 = 0.5,
    half_height: f32 = 1.0
);
impl R3DCollider for R3DCapsuleCollider {
    fn get_shape(&self) -> SharedShape {
        SharedShape::capsule_y(self.half_height, self.radius)
    }

    fn get_debug_mesh(&self) -> Gd<Mesh> {
        let mut mesh = CapsuleMesh::new_gd();
        mesh.set_height(self.half_height * 2.0 + self.radius * 2.0);
        mesh.set_radius(self.radius);
        mesh.upcast()
    }
}

collider!(
    R3DCuboidCollider,
    dimensions: Vector3 = Vector3::new(0.5, 0.5, 0.5)
);
impl R3DCollider for R3DCuboidCollider {
    fn get_shape(&self) -> SharedShape {
        SharedShape::cuboid(
            self.dimensions.x / 2.,
            self.dimensions.y / 2.,
            self.dimensions.z / 2.,
        )
    }

    fn get_debug_mesh(&self) -> Gd<Mesh> {
        let mut mesh = BoxMesh::new_gd();
        mesh.set_size(self.dimensions);
        mesh.upcast()
    }
}

collider!(
    R3DCylinderCollider,
    radius: f32 = 0.5,
    half_height: f32 = 1.0
);
impl R3DCollider for R3DCylinderCollider {
    fn get_shape(&self) -> SharedShape {
        SharedShape::cylinder(self.half_height, self.radius)
    }

    fn get_debug_mesh(&self) -> Gd<Mesh> {
        let mut mesh = CylinderMesh::new_gd();
        mesh.set_top_radius(self.radius);
        mesh.set_bottom_radius(self.radius);
        mesh.set_height(self.half_height * 2.0);
        mesh.upcast()
    }
}
