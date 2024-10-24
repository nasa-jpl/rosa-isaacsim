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
from omni.timeline import get_timeline_interface
from omni.stats import get_stats_interface
from omni.resourcemonitor import acquire_resource_monitor_interface
from omni.gpu_foundation_factory import get_gpu_foundation_factory_interface
from pprint import pprint
from omni.isaac.core.utils.prims import get_all_matching_child_prims

timeline = get_timeline_interface()
stats = get_stats_interface()
resource_monitor = acquire_resource_monitor_interface()
gpu_foundation = get_gpu_foundation_factory_interface()

prim_path = "/World/Carter_ROS"

update_output_box = None

def set_update_output_box(update_box_ref):
    global update_output_box
    update_output_box = update_box_ref

@tool
async def start_timeline():
    """
    Start the timeline.
    """
    global update_output_box
    update_output_box.text = "Starting timeline..."
    timeline.play()
    return "Timeline started."


@tool
async def stop_timeline():
    """
    Stop the timeline.
    """
    global update_output_box
    update_output_box.text = "Stopping timeline..."
    if timeline.is_playing():
        timeline.stop()
        return "Timeline stopped."
    else:
        return "Timeline is already stopped."


@tool
async def pause_timeline():
    """
    Pause the timeline.
    """
    global update_output_box
    update_output_box.text = "Pausing timeline..."
    if timeline.is_playing():
        timeline.pause()
        return "Timeline paused."
    else:
        return "Timeline is already paused."


@tool
async def is_timeline_playing():
    """
    Check if the timeline is currently playing.
    """
    return timeline.is_playing()


@tool
async def is_timeline_stopped():
    """
    Check if the timeline is currently stopped.
    """
    return timeline.is_stopped()


@tool
async def get_simulator_stats_and_diagnostics():
    """
    Get the simulator stats.
    """
    global update_output_box
    update_output_box.text = "Getting simulator stats and diagnostics..."
    scopes = stats.get_scopes()

    stat_values = {}

    for scope in scopes:
        st = stats.get_stats(scope["scopeId"])

        scope_values = []
        for stat in st:
            scope_values.append(
                {
                    "name": stat["name"],
                    "value": stat["value"],
                    "description": stat["description"],
                }
            )
        stat_values[scope["name"]] = scope_values

    available_device_memory = resource_monitor.get_available_device_memory(0)
    host_memory = resource_monitor.get_available_host_memory()
    total_device_memory = resource_monitor.get_total_device_memory(0)
    total_host_memory = resource_monitor.get_total_host_memory()

    result = {
        "agent_instructions": "The stats are provided below. Please do your best to interpret them and provide a summary. Do not use Markdown since the output box does not support it.",
        "stats": stat_values,
        "resources": {
            "available_device_memory": available_device_memory,
            "host_memory": host_memory,
            "total_device_memory": total_device_memory,
            "total_host_memory": total_host_memory,
            "device_diff": total_device_memory - available_device_memory,
            "host_diff": total_host_memory - host_memory,
            "is_device_overloaded": available_device_memory < 0.1 * total_device_memory,
            "is_host_overloaded": host_memory < 0.1 * total_host_memory,
        },
    }
    pprint(result)
    return result


@tool
async def get_carter_prims():
    """
    Get the details of the Carter robot.
    """
    global update_output_box
    update_output_box.text = "Getting Carter robot prims..."

    try:
        all_prims = get_all_matching_child_prims(prim_path=prim_path)
    except IndexError as e:
        return {"error": f"The carter robot does not appear to be loaded in the scene at the moment."}

    carter_prim = all_prims[0]
    return all_prims


@tool
async def get_all_prims():
    """
    Get all prims in the scene.
    """
    global update_output_box
    update_output_box.text = "Getting all prims..."

    all_prims = get_all_matching_child_prims(prim_path="/")
    return all_prims