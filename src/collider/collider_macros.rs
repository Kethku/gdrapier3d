use godot::engine::Mesh;
use godot::prelude::*;
use rapier3d::prelude::*;

pub trait R3DCollider {
    fn get_shape(&self) -> Option<SharedShape>;
    fn get_debug_mesh(&self) -> Gd<Mesh>;
}

#[macro_export]
macro_rules! collider {
    ($type_name:ident, $(#[export] $field_names:ident: $field_types:ty = $field_default:expr),+,) => {
        #[derive(GodotClass)]
        #[class(tool, base = Node3D)]
        pub struct $type_name {
            $(
                #[var(get, set = $field_names)]
                #[export]
                $field_names: $field_types,
            )+
            #[export]
            restitution: f32,
            #[export]
            density: f32,

            handle: Option<ColliderHandle>,
            mesh_instance: Option<Gd<MeshInstance3D>>,

            node_3d: Base<Node3D>,
        }

        #[godot_api]
        impl $type_name {
            fn setup_debug_mesh(&mut self) {
                let mut mesh_instance = MeshInstance3D::new_alloc();
                mesh_instance.set_name("collider_editor_mesh".into());
                self.base_mut().add_child(mesh_instance.clone().upcast::<Node>());
                self.mesh_instance = Some(mesh_instance);
            }

            fn update_debug_mesh(&mut self) {
                if let Some(mesh_instance) = self.mesh_instance.clone().as_mut() {
                    if let Ok(material) = try_load::<godot::engine::StandardMaterial3D>("res://utils/Collider.tres") {
                        mesh_instance.set_mesh(self.get_debug_mesh());
                        for surface in 0..mesh_instance.get_surface_override_material_count() {
                            mesh_instance.set_surface_override_material(surface, material.clone().upcast::<godot::engine::Material>());
                        }
                    }
                }
            }

            fn register_collider(&mut self) {
                // TODO: This needs to not panic. Should display this as a warning.
                let mut body = self.base().get_parent().unwrap().cast::<R3DRigidBody>();

                let translation = self.base().get_position();
                let rotation = self.base().get_rotation();

                if let Some(shape) = self.get_shape() {
                    self.handle = {
                        let mut body = body.bind_mut();
                        body.add_collider(
                            ColliderBuilder::new(shape)
                                .translation(vector![translation.x, translation.y, translation.z])
                                .rotation(vector![rotation.x, rotation.y, rotation.z])
                                .restitution(self.restitution)
                                .density(self.density)
                                .build(),
                        )
                    };
                } else {
                    eprintln!("Collider did not return a valid shape");
                }
            }

            fn unregister_collider(&mut self) {
                if let Some(mut body_gd) = self
                    .base()
                    .get_parent()
                    .and_then(|p| p.try_cast::<R3DRigidBody>().ok())
                {
                    if let Some(handle) = self.handle {
                        let mut body = body_gd.bind_mut();
                        body.remove_collider(handle);
                    }
                } else {
                    godot_print!("Collider parent is not a rigid body");
                }
            }

            $(
                #[func]
                pub fn $field_names(&mut self, $field_names: $field_types) {
                    self.$field_names = $field_names;
                    let editor = Engine::singleton().is_editor_hint();
                    if editor {
                        self.update_debug_mesh();
                    }
                }
            )+
        }

        #[godot_api]
        impl INode3D for $type_name {
            fn init(node_3d: Base<Node3D>) -> Self {
                Self {
                    $(
                        $field_names: $field_default,
                    )+
                    restitution: 0.0,
                    density: 1.0,
                    handle: None,
                    mesh_instance: None,
                    node_3d,
                }
            }

            fn ready(&mut self) {
                if Engine::singleton().is_editor_hint() {
                    self.setup_debug_mesh();
                    self.update_debug_mesh();
                }
            }

            fn enter_tree(&mut self) {
                self.register_collider();
            }

            fn exit_tree(&mut self) {
                self.unregister_collider();
            }
        }
    };
}
