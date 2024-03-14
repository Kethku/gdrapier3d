use godot::engine::ImmediateMesh;
use rapier3d::prelude::*;

pub trait R3DCollider {
    fn get_shape(&self) -> SharedShape;
    fn draw_debug_geometry(&self, _immediate_mesh: &mut ImmediateMesh) {}
}

#[macro_export]
macro_rules! collider {
    ($type_name:ident, $($field_names:ident: $field_types:ident = $field_default:expr),+) => {
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

            handle: Option<ColliderHandle>,

            node_3d: Base<Node3D>,
        }

        #[godot_api]
        impl $type_name {
            fn setup_immediate_mesh(&mut self) {
                let mut mesh_instance = MeshInstance3D::new_alloc();
                let immediate_mesh = ImmediateMesh::new_gd().upcast::<Mesh>();
                mesh_instance.set_mesh(immediate_mesh);
                mesh_instance.set_name("collider_editor_mesh".into());
                self.base_mut().add_child(mesh_instance.upcast::<Node>());
            }

            fn update_immediate_mesh(&mut self) {
                if let Some(mesh_instance) = self.base()
                    .find_child_ex("collider_editor_mesh".into())
                    .owned(false)
                    .done()
                {
                    let mesh_instance = mesh_instance.cast::<MeshInstance3D>();

                    if let Some(immediate_mesh) = mesh_instance.get_mesh() {
                        let mut immediate_mesh = immediate_mesh.cast::<ImmediateMesh>();
                        immediate_mesh.clear_surfaces();
                        immediate_mesh.call(
                            "surface_begin".into(),
                            &[
                                Variant::from(PrimitiveType::LINES),
                                Variant::nil(),
                            ],
                        );
                        self.draw_debug_geometry(&mut immediate_mesh);
                        immediate_mesh.surface_end();
                    }
                }
            }

            fn register_collider(&mut self) {
                // TODO: This needs to not panic. Should display this as a warning.
                let mut body = self.base().get_parent().unwrap().cast::<R3DRigidBody>();

                let translation = self.base().get_position();
                let rotation = self.base().get_rotation();

                self.handle = {
                    let mut body = body.bind_mut();
                    body.add_collider(
                        ColliderBuilder::new(self.get_shape())
                            .translation(vector![translation.x, translation.y, translation.z])
                            .rotation(vector![rotation.x, rotation.y, rotation.z])
                            .restitution(self.restitution)
                            .build(),
                    )
                };
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
                    let mut this = self.base().clone().cast::<Self>();
                    if editor {
                        this.call_deferred("update_immediate_mesh".into(), &[]);
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
                    handle: None,
                    node_3d,
                }
            }

            fn ready(&mut self) {
                if Engine::singleton().is_editor_hint() {
                    self.setup_immediate_mesh();
                    self.update_immediate_mesh();
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
