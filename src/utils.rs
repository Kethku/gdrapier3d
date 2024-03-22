use godot::prelude::*;
use rapier3d::prelude::{Isometry, Real};

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
