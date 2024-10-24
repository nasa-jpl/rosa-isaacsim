#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from langchain.agents import tool
import omni.usd
import omni.isaac.core.utils.stage as stage_utils

update_output_box = None

def set_update_output_box(update_box_ref):
    global update_output_box
    update_output_box = update_box_ref


isaac_scenes = {}

@tool
async def get_available_usd_scenes():
    """
    Get all available USD scenes for the current IsaacSim environment.
    """
    return isaac_scenes


@tool
async def load_usd_scene(scene_code: str):
    """
    Load the specified USD scene in the IsaacSim environment.

    :param scene_code: The code (index) of the scene to load. Must be one of the available scenes.
    """
    print(f"Loading the {scene_code} scene...")
    global update_output_box
    update_output_box.text = f"Adding reference to scene: {scene_code}..."

    if scene_code not in isaac_scenes:
        return {"error": f"Scene '{scene_code}' not found. Use the 'get_available_usd_scenes' tool to get a list of available scenes."}

    scene = isaac_scenes[scene_code]
    usd_path = scene["path"]
    
    # Load the USD stage (scene) and timeline
    stage_utils.add_reference_to_stage(usd_path, "/World")
    update_output_box.text = f"Loaded the '{scene_code}' scene."
    

    return f"Loaded the {scene_code} scene successfully. Scene description: {scene['description']}"
