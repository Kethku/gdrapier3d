mod collider_macros;

use godot::{
    engine::{
        mesh::ArrayType, BoxMesh, CapsuleMesh, CylinderMesh, Engine, Mesh, MeshInstance3D,
        SphereMesh,
    },
    obj::IndexEnum,
    prelude::*,
};
use rapier3d::{na::Point3, prelude::*};

use crate::{collider, rigid_body::R3DRigidBody};

pub use self::collider_macros::R3DCollider;

collider!(
    R3DBallCollider,
    #[export]
    radius: f32 = 0.5,
);
impl R3DCollider for R3DBallCollider {
    fn get_shape(&self, scale: f32) -> Option<SharedShape> {
        Some(SharedShape::ball(self.radius * scale))
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
    #[export]
    radius: f32 = 0.5,
    #[export]
    half_height: f32 = 1.0,
);
impl R3DCollider for R3DCapsuleCollider {
    fn get_shape(&self, scale: f32) -> Option<SharedShape> {
        Some(SharedShape::capsule_y(
            self.half_height * scale,
            self.radius * scale,
        ))
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
    #[export]
    dimensions: Vector3 = Vector3::new(0.5, 0.5, 0.5),
);
impl R3DCollider for R3DCuboidCollider {
    fn get_shape(&self, scale: f32) -> Option<SharedShape> {
        Some(SharedShape::cuboid(
            scale * self.dimensions.x / 2.,
            scale * self.dimensions.y / 2.,
            scale * self.dimensions.z / 2.,
        ))
    }

    fn get_debug_mesh(&self) -> Gd<Mesh> {
        let mut mesh = BoxMesh::new_gd();
        mesh.set_size(self.dimensions);
        mesh.upcast()
    }
}

collider!(
    R3DCylinderCollider,
    #[export]
    radius: f32 = 0.5,
    #[export]
    half_height: f32 = 1.0,
);
impl R3DCollider for R3DCylinderCollider {
    fn get_shape(&self, scale: f32) -> Option<SharedShape> {
        Some(SharedShape::cylinder(
            self.half_height * scale,
            self.radius * scale,
        ))
    }

    fn get_debug_mesh(&self) -> Gd<Mesh> {
        let mut mesh = CylinderMesh::new_gd();
        mesh.set_top_radius(self.radius);
        mesh.set_bottom_radius(self.radius);
        mesh.set_height(self.half_height * 2.0);
        mesh.upcast()
    }
}

collider!(
    R3DMeshCollider,
    #[export]
    mesh: Gd<Mesh> = Mesh::new_gd(),
);
impl R3DCollider for R3DMeshCollider {
    fn get_shape(&self, scale: f32) -> Option<SharedShape> {
        if self.mesh.get_surface_count() == 0 {
            return None;
        }

        let arrays = self.mesh.surface_get_arrays(0);

        let vertices = arrays
            .get(ArrayType::VERTEX.to_index())
            .clone()
            .to::<PackedVector3Array>()
            .as_slice()
            .into_iter()
            .map(|v| *v * scale)
            .map(|v| Point3::new(v.x, v.y, v.z))
            .collect::<Vec<_>>();

        let mut indices_triplets = Vec::new();
        let indices_array = arrays
            .get(ArrayType::INDEX.to_index())
            .clone()
            .to::<PackedInt32Array>();
        let mut indices_iter = indices_array.as_slice().into_iter();
        while let Some(next) = indices_iter.next() {
            let i0 = *next as u32;
            let i1 = *indices_iter.next().unwrap() as u32;
            let i2 = *indices_iter.next().unwrap() as u32;
            indices_triplets.push([i0, i1, i2]);
        }

        Some(SharedShape::trimesh(vertices, indices_triplets))
    }

    fn get_debug_mesh(&self) -> Gd<Mesh> {
        self.mesh.clone()
    }
}
