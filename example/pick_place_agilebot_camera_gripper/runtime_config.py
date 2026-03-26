import json
import os
import tempfile
from functools import lru_cache


REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
CONFIG_PATH = os.path.join(REPO_ROOT, "config", "robot_config.json")


@lru_cache(maxsize=1)
def load_robot_config() -> dict:
    with open(CONFIG_PATH, "r", encoding="utf-8") as config_file:
        return json.load(config_file)


def get_repo_path(relative_path: str) -> str:
    return os.path.join(REPO_ROOT, relative_path)


def get_robot_root_prim_path() -> str:
    return load_robot_config()["scene"]["robot_root_prim_path"]


def get_end_effector_frame_name() -> str:
    return load_robot_config()["robot"]["end_effector_frame_name"]


def get_end_effector_prim_name_candidates() -> list[str]:
    return load_robot_config()["robot"]["end_effector_prim_name_candidates"]


def get_gripper_joint_prim_names() -> list[str]:
    return load_robot_config()["robot"]["gripper_joint_prim_names"]


def get_camera_prim_name_candidates() -> list[str]:
    return load_robot_config()["camera"]["prim_name_candidates"]


def get_camera_resolution() -> tuple[int, int]:
    width, height = load_robot_config()["camera"]["resolution"]
    return width, height


def get_robot_description_path() -> str:
    return get_repo_path(load_robot_config()["assets"]["robot_description_path"])


def get_urdf_path() -> str:
    return get_repo_path(load_robot_config()["assets"]["urdf_path"])


def get_usd_candidate_paths() -> list[str]:
    return [
        get_repo_path(relative_path)
        for relative_path in load_robot_config()["assets"]["usd_candidates"]
    ]


def build_rmpflow_config_path() -> str:
    template_path = get_repo_path(load_robot_config()["assets"]["rmpflow_template_path"])
    with open(template_path, "r", encoding="utf-8") as template_file:
        template_text = template_file.read()

    rendered_text = template_text.replace(
        "__END_EFFECTOR_FRAME_NAME__",
        get_end_effector_frame_name(),
    )

    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".yaml",
        prefix="gbt_c5a_rmpflow_",
        delete=False,
    ) as rendered_file:
        rendered_file.write(rendered_text)
        return rendered_file.name
