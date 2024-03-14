use godot::{
    builtin::meta::{ConvertError, GodotConvert},
    prelude::*,
};

use crate::rigid_body::R3DRigidBody;

#[derive(Debug)]
pub enum RaycastResult {
    None,
    Hit {
        body: Gd<R3DRigidBody>,
        distance: f32,
    },
}

impl GodotConvert for RaycastResult {
    type Via = Dictionary;
}

impl ToGodot for RaycastResult {
    fn to_godot(&self) -> Self::Via {
        let mut dictionary = Dictionary::new();
        if let RaycastResult::Hit { body, distance } = self {
            dictionary.insert("body", body.clone());
            dictionary.insert("distance", *distance);
        }
        dictionary
    }
}

impl FromGodot for RaycastResult {
    fn try_from_godot(_via: Self::Via) -> Result<Self, ConvertError> {
        unimplemented!();
    }
}
