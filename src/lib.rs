pub mod collider;
pub mod raycast_result;
pub mod rigid_body;
pub mod utils;
pub mod world;

use godot::{
    engine::{EditorPlugin, IEditorPlugin, ResourceSaver},
    prelude::*,
};

use world::R3DWorld;

struct GdRapier3d;

#[gdextension]
unsafe impl ExtensionLibrary for GdRapier3d {}

#[derive(GodotClass)]
#[class(tool, editor_plugin, base=EditorPlugin)]
struct GdRapier3dEditorPlugin {
    base: Base<EditorPlugin>,
}

#[godot_api]
impl IEditorPlugin for GdRapier3dEditorPlugin {
    fn init(base: Base<EditorPlugin>) -> Self {
        GdRapier3dEditorPlugin { base }
    }

    fn enter_tree(&mut self) {
        let project_settings = ProjectSettings::singleton();
        let directory_string: String = project_settings
            .globalize_path("res://autoloads".into())
            .into();
        let directory_path = PathBuf::from(directory_string);

        std::fs::create_dir_all(directory_path).expect("Couldn't create autoloads directory");

        let autoloads: Vec<(GString, GString, Gd<Node>)> = vec![(
            "World".into(),
            "res://autoloads/world.tscn".into(),
            R3DWorld::new_alloc().upcast::<Node>(),
        )];

        for (name, path, instance) in autoloads.into_iter() {
            let mut resource_saver = ResourceSaver::singleton();
            let mut packed_scene = PackedScene::new_gd();
            packed_scene.pack(instance);
            resource_saver
                .save_ex(packed_scene.upcast())
                .path(path.clone())
                .done();
            self.base_mut().add_autoload_singleton(name, path);
        }
    }
}
