use std::collections::HashMap;

use godot::prelude::*;
use rapier3d::{
    control::{EffectiveCharacterMovement, KinematicCharacterController},
    na::{Isometry3, UnitQuaternion},
    prelude::*,
};

use crate::{
    raycast_result::RaycastResult,
    rigid_body::{BodyType, R3DRigidBody},
    utils::isometry_to_transform,
};

#[derive(Clone)]
struct WorldState {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,

    // Simulation structures
    gravity: Vector3,
    integration_parameters: IntegrationParameters,
    query_pipeline: QueryPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,

    // State
    godot_body_node_lookup: HashMap<RigidBodyHandle, String>,
    body_handle_lookup: HashMap<String, RigidBodyHandle>,
}

#[derive(GodotClass)]
#[class(base = Node)]
pub struct R3DWorld {
    current_tick: u32,
    frames: HashMap<u32, WorldState>,
    physics_pipeline: PhysicsPipeline,

    node: Base<Node>,
}

#[godot_api]
impl INode for R3DWorld {
    fn init(node: Base<Node>) -> Self {
        let mut frames = HashMap::new();
        frames.insert(
            0,
            WorldState {
                rigid_body_set: RigidBodySet::new(),
                collider_set: ColliderSet::new(),

                // Simulation structures
                gravity: Vector3::new(0., -9.81, 0.),
                integration_parameters: IntegrationParameters::default(),
                query_pipeline: QueryPipeline::new(),
                island_manager: IslandManager::new(),
                broad_phase: BroadPhase::new(),
                narrow_phase: NarrowPhase::new(),
                impulse_joint_set: ImpulseJointSet::new(),
                multibody_joint_set: MultibodyJointSet::new(),
                ccd_solver: CCDSolver::new(),

                // State
                godot_body_node_lookup: HashMap::new(),
                body_handle_lookup: HashMap::new(),
            },
        );

        Self {
            current_tick: 0,
            frames,
            physics_pipeline: PhysicsPipeline::new(),

            node,
        }
    }

    fn ready(&mut self) {
        self.base_mut().add_to_group("networked".into());
    }
}

#[godot_api]
impl R3DWorld {
    #[func]
    pub fn load_state(&mut self, tick: u32) {
        self.current_tick = tick;
        self.frames.retain(|k, _| *k <= self.current_tick);
    }

    #[func]
    pub fn networked_preprocess(&self) {
        let frame = self.frames.get(&self.current_tick).unwrap();
        for (handle, body) in frame.rigid_body_set.iter() {
            if let Some(godot_node_path) = frame.godot_body_node_lookup.get(&handle) {
                if let Some(mut godot_node) = self
                    .base()
                    .get_node(godot_node_path.into())
                    .map(|node| node.cast::<R3DRigidBody>())
                {
                    let transform = isometry_to_transform(body.position());
                    godot_node.set_transform(transform);
                }
            }
        }
    }

    #[func]
    pub fn networked_process(&mut self) -> u32 {
        let mut frame = self.frames.get_mut(&self.current_tick).unwrap().clone();
        self.physics_pipeline.step(
            &vector![frame.gravity.x, frame.gravity.y, frame.gravity.z],
            &frame.integration_parameters,
            &mut frame.island_manager,
            &mut frame.broad_phase,
            &mut frame.narrow_phase,
            &mut frame.rigid_body_set,
            &mut frame.collider_set,
            &mut frame.impulse_joint_set,
            &mut frame.multibody_joint_set,
            &mut frame.ccd_solver,
            None,
            &(),
            &(),
        );
        frame
            .query_pipeline
            .update(&frame.rigid_body_set, &frame.collider_set);
        for (handle, body) in frame.rigid_body_set.iter_mut() {
            body.reset_forces(true);
            body.reset_torques(true);

            if let Some(godot_node_path) = frame.godot_body_node_lookup.get(&handle) {
                if let Some(mut godot_node) = self
                    .base()
                    .get_node(godot_node_path.into())
                    .map(|node| node.cast::<R3DRigidBody>())
                {
                    let transform = isometry_to_transform(body.position());
                    godot_node.set_transform(transform);
                } else {
                    let mut sync_manager =
                        self.base().get_node("/root/SyncManager".into()).unwrap();
                    sync_manager.call(
                        "log".into(),
                        &[format!(
                            "Tracked Node Not Found on tick {} {handle:?}",
                            self.current_tick
                        )
                        .to_variant()],
                    );
                    panic!("Tracked node not found. Maybe it was deleted without chance to unregister?");
                }
            }
        }

        self.current_tick += 1;
        self.frames.insert(self.current_tick, frame);
        self.current_tick
    }

    #[func]
    pub fn bodies_within_sphere(&self, position: Vector3, radius: f32) -> Array<Gd<R3DRigidBody>> {
        let frame = self.frames.get(&self.current_tick).unwrap();

        let mut intersecting_colliders = Vec::new();
        frame.query_pipeline.intersections_with_shape(
            &frame.rigid_body_set,
            &frame.collider_set,
            &Isometry3::<f32>::translation(position.x, position.y, position.z),
            &Ball::new(radius),
            QueryFilter::only_dynamic(),
            |collider| {
                intersecting_colliders.push(collider);
                true
            },
        );

        intersecting_colliders
            .into_iter()
            .map(|collider_handle| {
                let collider = frame
                    .collider_set
                    .get(collider_handle)
                    .expect("Collider missing");
                let body = collider.parent().expect("Collider has no parent");
                let node_path = frame
                    .godot_body_node_lookup
                    .get(&body)
                    .expect("Body not found");
                self.base()
                    .get_node(node_path.into())
                    .expect("Node not found")
                    .cast::<R3DRigidBody>()
            })
            .collect()
    }
}

impl R3DWorld {
    pub fn body<'a>(&'a self, handle: RigidBodyHandle) -> Option<&'a RigidBody> {
        let frame = self.frames.get(&self.current_tick).unwrap();
        frame.rigid_body_set.get(handle)
    }

    pub fn body_mut<'a>(&'a mut self, handle: RigidBodyHandle) -> Option<&'a mut RigidBody> {
        let frame = self.frames.get_mut(&self.current_tick).unwrap();
        frame.rigid_body_set.get_mut(handle)
    }

    pub fn add_body(&mut self, godot_node: &mut R3DRigidBody) -> RigidBodyHandle {
        let frame = self.frames.get_mut(&self.current_tick).unwrap();
        let node_path = godot_node.base().get_path().to_string();
        if let Some(handle) = frame.body_handle_lookup.get(&node_path) {
            // Body already exists, pull the existing handle and update node position
            let body = frame.rigid_body_set.get(*handle).unwrap();
            let transform = isometry_to_transform(body.position());
            godot_node.base_mut().set_transform(transform);
            *handle
        } else {
            // Body does not exist. Create a new one and set its position
            let body_type = match godot_node.body_type {
                BodyType::Dynamic => RigidBodyType::Dynamic,
                BodyType::Fixed => RigidBodyType::Fixed,
                BodyType::Kinematic => RigidBodyType::KinematicPositionBased,
            };
            let translation = godot_node.base().get_position();
            let rotation = godot_node.base().get_rotation();
            let position = Isometry::<Real>::from_parts(
                Translation::new(translation.x, translation.y, translation.z),
                UnitQuaternion::from_euler_angles(rotation.x, rotation.y, rotation.z),
            );
            let body = RigidBodyBuilder::new(body_type).position(position).build();
            let handle = frame.rigid_body_set.insert(body);
            frame
                .godot_body_node_lookup
                .insert(handle, node_path.clone());
            frame.body_handle_lookup.insert(node_path, handle);
            let mut sync_manager = self.base().get_node("/root/SyncManager".into()).unwrap();
            sync_manager.call(
                "log".into(),
                &[format!("Added Body on Tick {} {handle:?}", self.current_tick).to_variant()],
            );
            handle
        }
    }

    pub fn remove_body(&mut self, handle: RigidBodyHandle) {
        let mut sync_manager = self.base().get_node("/root/SyncManager".into()).unwrap();
        sync_manager.call(
            "log".into(),
            &[format!("Removing Body {handle:?}").to_variant()],
        );
        let frame = self.frames.get_mut(&self.current_tick).unwrap();
        frame.godot_body_node_lookup.remove(&handle);
        frame.rigid_body_set.remove(
            handle,
            &mut frame.island_manager,
            &mut frame.collider_set,
            &mut frame.impulse_joint_set,
            &mut frame.multibody_joint_set,
            false,
        );

        sync_manager.call(
            "log".into(),
            &[format!("Removed Body on Tick {} {handle:?}", self.current_tick).to_variant()],
        );
    }

    pub fn add_collider_to_body(
        &mut self,
        collider: Collider,
        body: RigidBodyHandle,
    ) -> ColliderHandle {
        let frame = self.frames.get_mut(&self.current_tick).unwrap();
        frame
            .collider_set
            .insert_with_parent(collider, body, &mut frame.rigid_body_set)
    }

    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Option<Collider> {
        let frame = self.frames.get_mut(&self.current_tick).unwrap();
        frame.collider_set.remove(
            handle,
            &mut frame.island_manager,
            &mut frame.rigid_body_set,
            true,
        )
    }

    pub fn move_shape(
        &self,
        dt: f32,
        character_shape: &dyn Shape,
        character_pos: &Isometry<f32>,
        desired_translation: Vector<f32>,

        body_handle: RigidBodyHandle,
    ) -> EffectiveCharacterMovement {
        let frame = self.frames.get(&self.current_tick).unwrap();
        let character_controller = KinematicCharacterController::default();
        character_controller.move_shape(
            dt,
            &frame.rigid_body_set,
            &frame.collider_set,
            &frame.query_pipeline,
            character_shape,
            character_pos,
            desired_translation,
            QueryFilter::default().exclude_rigid_body(body_handle),
            |_| {},
        )
    }

    pub fn raycast(
        &self,
        body: RigidBodyHandle,
        direction: Vector<f32>,
        max_distance: f32,
    ) -> RaycastResult {
        let frame = self.frames.get(&self.current_tick).unwrap();
        let position = frame.rigid_body_set.get(body).unwrap().translation();
        let ray = Ray::new(
            Point::new(position.x, position.y, position.z),
            direction.normalize(),
        );

        let Some((collider, distance)) = frame.query_pipeline.cast_ray(
            &frame.rigid_body_set,
            &frame.collider_set,
            &ray,
            max_distance,
            true,
            QueryFilter::default().exclude_rigid_body(body),
        ) else {
            return RaycastResult::None;
        };

        let collider = frame
            .collider_set
            .get(collider)
            .expect("Collider not found");
        let body = collider.parent().expect("Collider has no parent");
        let node_path = frame
            .godot_body_node_lookup
            .get(&body)
            .expect("Body not found");
        let node = self
            .base()
            .get_node(node_path.into())
            .expect("Node not found")
            .cast::<R3DRigidBody>();
        RaycastResult::Hit {
            body: node.clone(),
            distance,
        }
    }
}
