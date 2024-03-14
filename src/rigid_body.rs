use godot::prelude::*;
use rapier3d::prelude::*;

use crate::{raycast_result::RaycastResult, world::R3DWorld};

#[derive(GodotConvert, Export, Var, PartialEq, Eq)]
#[godot(via = u8)]
pub enum BodyType {
    Dynamic,
    Fixed,
    Kinematic,
}

#[derive(GodotClass)]
#[class(base = Node3D)]
pub struct R3DRigidBody {
    #[export]
    pub body_type: BodyType,

    world: Option<Gd<R3DWorld>>,
    handle: Option<RigidBodyHandle>,
    compound_shape: Option<SharedShape>,

    pub node_3d: Base<Node3D>,
}

#[godot_api]
impl INode3D for R3DRigidBody {
    fn init(node_3d: Base<Node3D>) -> Self {
        Self {
            body_type: BodyType::Dynamic,
            world: None,
            handle: None,
            compound_shape: None,
            node_3d,
        }
    }

    fn ready(&mut self) {
        let mut node_3d = self.base_mut();
        node_3d.set_process(true);
        node_3d.set_rotation_order(godot::engine::global::EulerOrder::ZXY);
        node_3d.add_to_group("networked".into());
    }

    fn enter_tree(&mut self) {
        self.register_body();
    }

    fn exit_tree(&mut self) {
        self.unregister_body();
    }
}

#[godot_api]
impl R3DRigidBody {
    pub fn add_collider(&mut self, collider: Collider) -> Option<ColliderHandle> {
        if let Some(world) = self.world.as_mut() {
            let mut world = world.bind_mut();
            let handle = world.add_collider_to_body(collider, self.handle.clone().unwrap());
            self.compound_shape = world.compound_shape(self.handle.unwrap());
            Some(handle)
        } else {
            None
        }
    }

    pub fn remove_collider(&mut self, collider_handle: ColliderHandle) -> Option<Collider> {
        if let Some(world) = self.world.as_mut() {
            let mut world = world.bind_mut();
            self.compound_shape = world.compound_shape(self.handle.unwrap());
            world.remove_collider(collider_handle)
        } else {
            None
        }
    }

    fn register_body(&mut self) {
        let mut world = self
            .base()
            .get_node("/root/World".into())
            .unwrap()
            .cast::<R3DWorld>();

        self.handle = Some(world.bind_mut().add_body(self));
        self.world = Some(world);
    }

    fn unregister_body(&mut self) {
        if let Some(handle) = self.handle.take() {
            let mut world_gd = self.world.take().unwrap();
            let mut world = world_gd.bind_mut();
            world.remove_body(handle);
        }
    }

    #[func]
    fn networked_despawn(&mut self) {
        let mut sync_manager = self.base().get_node("/root/SyncManager".into()).unwrap();
        sync_manager.call("log".into(), &["Networked despawn called".to_variant()]);
        self.unregister_body();
    }

    #[func]
    fn raycast(&self, direction: Vector3, max_distance: f32) -> RaycastResult {
        let handle = self.handle.unwrap();
        let mut direction = vector![direction.x, direction.y, direction.z];
        direction = direction.normalize();
        self.world
            .as_ref()
            .unwrap()
            .bind()
            .raycast(handle, direction, max_distance)
    }

    #[func]
    fn add_force(&mut self, force: Vector3) {
        let handle = self.handle.unwrap();
        let mut world = self.world.as_mut().unwrap().bind_mut();
        let body = world.body_mut(handle).unwrap();
        body.add_force(vector![force.x, force.y, force.z], true);
    }

    #[func]
    fn add_torque(&mut self, torque: Vector3) {
        let handle = self.handle.unwrap();
        let mut world = self.world.as_mut().unwrap().bind_mut();
        let body = world.body_mut(handle).unwrap();
        body.add_torque(vector![torque.x, torque.y, torque.z], true);
    }

    #[func]
    fn apply_impulse(&mut self, impulse: Vector3) {
        let handle = self.handle.unwrap();
        let mut world = self.world.as_mut().unwrap().bind_mut();
        let body = world.body_mut(handle).unwrap();
        body.apply_impulse(vector![impulse.x, impulse.y, impulse.z], true);
    }

    #[func]
    fn apply_torque_impulse(&mut self, torque_impulse: Vector3) {
        let handle = self.handle.unwrap();
        let mut world = self.world.as_mut().unwrap().bind_mut();
        let body = world.body_mut(handle).unwrap();
        body.apply_torque_impulse(
            vector![torque_impulse.x, torque_impulse.y, torque_impulse.z],
            true,
        );
    }

    #[func]
    fn mass(&self) -> f32 {
        let handle = self.handle.unwrap();
        let world = self.world.as_ref().unwrap().bind();
        let body = world.body(handle).unwrap();
        body.mass()
    }

    #[func]
    fn linear_velocity(&self) -> Vector3 {
        let handle = self.handle.unwrap();
        let world = self.world.as_ref().unwrap().bind();
        let body = world.body(handle).unwrap();
        let velocity = body.linvel();
        Vector3::new(velocity.x, velocity.y, velocity.z)
    }

    #[func]
    fn angular_velocity(&self) -> Vector3 {
        let handle = self.handle.unwrap();
        let world = self.world.as_ref().unwrap().bind();
        let body = world.body(handle).unwrap();
        let velocity = body.angvel();
        Vector3::new(velocity.x, velocity.y, velocity.z)
    }
}
