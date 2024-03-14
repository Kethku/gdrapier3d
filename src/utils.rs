use godot::{engine::ImmediateMesh, prelude::*};
use rapier3d::prelude::{Isometry, Real};

pub trait ImmediateMeshExt {
    fn add_line(&mut self, p1: Vector3, p2: Vector3);
    fn add_quad(&mut self, forward: Vector3, up: Vector3, right: Vector3);
    fn add_front_back(&mut self, forward: Vector3, up: Vector3, right: Vector3);
    fn add_edges(&mut self, forward: Vector3, up: Vector3, right: Vector3);
    fn add_cuboid(&mut self, dimensions: Vector3);
    fn add_circle(&mut self, center: Vector3, normal: Vector3, radius: f32);
    fn add_half_circle(
        &mut self,
        center: Vector3,
        normal: Vector3,
        arc_center: Vector3,
        radius: f32,
    );
    fn add_ball(&mut self, radius: f32);
    fn add_half_ball(&mut self, center: Vector3, normal: Vector3, radius: f32);
    fn add_cylinder(&mut self, radius: f32, half_height: f32);
    fn add_capsule(&mut self, radius: f32, half_height: f32);
}

impl ImmediateMeshExt for ImmediateMesh {
    fn add_line(&mut self, p1: Vector3, p2: Vector3) {
        self.surface_add_vertex(p1);
        self.surface_add_vertex(p2);
    }

    fn add_quad(&mut self, forward: Vector3, up: Vector3, right: Vector3) {
        let p1 = forward + up + right;
        let p2 = forward + up - right;
        let p3 = forward - up - right;
        let p4 = forward - up + right;
        self.add_line(p1, p2);
        self.add_line(p2, p3);
        self.add_line(p3, p4);
        self.add_line(p4, p1);
    }

    fn add_front_back(&mut self, forward: Vector3, up: Vector3, right: Vector3) {
        self.add_quad(forward, up, right);
        self.add_quad(-forward, up, right);
    }

    fn add_edges(&mut self, forward: Vector3, up: Vector3, right: Vector3) {
        let p1 = up + right;
        let p2 = up - right;
        let p3 = -up - right;
        let p4 = -up + right;
        self.add_line(p1 - forward, p1 + forward);
        self.add_line(p2 - forward, p2 + forward);
        self.add_line(p3 - forward, p3 + forward);
        self.add_line(p4 - forward, p4 + forward);
    }

    fn add_cuboid(&mut self, dimensions: Vector3) {
        let forward = Vector3::new(0.0, 0.0, 1.0) * dimensions.z / 2.0;
        let up = Vector3::new(0.0, 1.0, 0.0) * dimensions.y / 2.0;
        let right = Vector3::new(1.0, 0.0, 0.0) * dimensions.x / 2.0;
        self.add_front_back(forward, up, right);
        self.add_edges(forward, up, right);
    }

    fn add_circle(&mut self, center: Vector3, normal: Vector3, radius: f32) {
        let perpendicular = perpendicular(normal);

        let mut previous = perpendicular * radius + center;
        for r in 1..=32 {
            let rotation = r as f32 / 32.0 * std::f32::consts::TAU;
            let next = perpendicular.rotated(normal, rotation) * radius + center;
            self.add_line(previous, next);
            previous = next;
        }
    }

    fn add_half_circle(
        &mut self,
        center: Vector3,
        normal: Vector3,
        arc_center: Vector3,
        radius: f32,
    ) {
        let perpendicular = arc_center.cross(normal).normalized();
        let mut previous = perpendicular * radius + center;
        for r in 1..=16 {
            let rotation = r as f32 / 16.0 * std::f32::consts::PI;
            let next = perpendicular.rotated(normal, rotation) * radius + center;
            self.add_line(previous, next);
            previous = next;
        }
    }

    fn add_ball(&mut self, radius: f32) {
        self.add_circle(Vector3::ZERO, Vector3::UP, radius);
        self.add_circle(Vector3::ZERO, Vector3::RIGHT, radius);
        self.add_circle(Vector3::ZERO, Vector3::FORWARD, radius);
    }

    fn add_half_ball(&mut self, center: Vector3, normal: Vector3, radius: f32) {
        let perpendicular = perpendicular(normal);
        self.add_half_circle(center, perpendicular, normal, radius);
        self.add_half_circle(center, normal.cross(perpendicular), normal, radius);
    }

    fn add_cylinder(&mut self, radius: f32, half_height: f32) {
        self.add_circle(Vector3::UP * half_height, Vector3::UP, radius);
        self.add_circle(Vector3::DOWN * half_height, Vector3::DOWN, radius);
        let perpendicular = Vector3::FORWARD;

        for r in 0..8 {
            let rotation = r as f32 / 8.0 * std::f32::consts::TAU;
            let center = perpendicular.rotated(Vector3::UP, rotation) * radius;
            self.add_line(
                center + Vector3::UP * half_height,
                center + Vector3::DOWN * half_height,
            );
        }
    }

    fn add_capsule(&mut self, radius: f32, half_height: f32) {
        self.add_half_ball(Vector3::UP * half_height, Vector3::UP, radius);
        self.add_cylinder(radius, half_height);
        self.add_half_ball(Vector3::DOWN * half_height, Vector3::DOWN, radius);
    }
}

pub fn perpendicular(v: Vector3) -> Vector3 {
    // Taking the crossproduct with the basis vectors will
    // will result in at minimum 2 non zero perpendicular vectors.
    // So we pick one and return it.
    // https://math.stackexchange.com/a/2724461

    let result = v.cross(Vector3::UP);
    if result.length() > 0.0001 {
        result.normalized()
    } else {
        v.cross(Vector3::RIGHT).normalized()
    }
}

pub fn isometry_to_transform(isometry: &Isometry<Real>) -> Transform3D {
    let matrix = isometry.to_matrix();
    let projection = Projection::new([
        Vector4::new(
            matrix[(0, 0)],
            matrix[(1, 0)],
            matrix[(2, 0)],
            matrix[(3, 0)],
        ),
        Vector4::new(
            matrix[(0, 1)],
            matrix[(1, 1)],
            matrix[(2, 1)],
            matrix[(3, 1)],
        ),
        Vector4::new(
            matrix[(0, 2)],
            matrix[(1, 2)],
            matrix[(2, 2)],
            matrix[(3, 2)],
        ),
        Vector4::new(
            matrix[(0, 3)],
            matrix[(1, 3)],
            matrix[(2, 3)],
            matrix[(3, 3)],
        ),
    ]);
    Transform3D::from_projection(projection)
}
