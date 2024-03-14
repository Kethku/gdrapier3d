mod collider_macros;

use godot::{
    engine::{mesh::PrimitiveType, Engine, ImmediateMesh, Mesh, MeshInstance3D},
    prelude::*,
};
use rapier3d::prelude::*;

use crate::{collider, rigid_body::R3DRigidBody, utils::ImmediateMeshExt};

pub use self::collider_macros::R3DCollider;

collider!(R3DBallCollider, radius: f32 = 0.5);
impl R3DCollider for R3DBallCollider {
    fn get_shape(&self) -> SharedShape {
        SharedShape::ball(self.radius)
    }

    fn draw_debug_geometry(&self, mesh: &mut ImmediateMesh) {
        mesh.add_ball(self.radius);
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

    fn draw_debug_geometry(&self, mesh: &mut ImmediateMesh) {
        mesh.add_capsule(self.radius, self.half_height);
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

    fn draw_debug_geometry(&self, mesh: &mut ImmediateMesh) {
        mesh.add_cuboid(self.dimensions);
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

    fn draw_debug_geometry(&self, mesh: &mut ImmediateMesh) {
        mesh.add_cylinder(self.radius, self.half_height);
    }
}
