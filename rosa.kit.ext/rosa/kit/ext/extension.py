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

import omni.ext
import omni.ui as ui

from datetime import datetime
import os
import sys
import time
import asyncio
import threading
import rospy
from geometry_msgs.msg import Twist

from .llm import get_llm
from .env import set_env_variables
from .rosa.src.rosa import ROSA
from .prompts import prompts

rospy.init_node("rosa_kit_ext")

set_env_variables()
llm = get_llm()
from .tools import carter_tools, timeline_tools, isaacsim_tools, camera_tools
from .tools.carter_tools import cmd_vel_pub, set_cmd_vel_pub, update_output_box, set_update_output_box, set_is_running

cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
set_cmd_vel_pub(cmd_vel_pub)
agent = ROSA(
    ros_version=1, 
    llm=llm, 
    tool_packages=[carter_tools, timeline_tools, isaacsim_tools, camera_tools], 
    prompts=prompts
)

# Create a new event loop in a separate thread
loop = asyncio.new_event_loop()
threading.Thread(target=loop.run_forever, daemon=True).start()

label = None
text_input = None
output_box = None
update_box = None
query = ""
running_tasks = []


def some_public_function(x: int):
    print("[rosa.kit.ext] some_public_function was called with x: ", x)
    return x ** x


def run_agent(query):
    return agent.astream(query)

async def update_output_box(output_box, async_gen):
    async for result in async_gen:
        if result["type"] == "token":
            output_box.text += result["content"]
        elif result["type"] == "final" or result["type"] == "error":
            print(datetime.now(), result)
            output_box.text = result["content"]
        else:
            print(datetime.now(), result)

        if result["type"] == "final":
            update_box.text = f"Query: {text_input.model.get_value_as_string()}"
        elif result["type"] == "error":
            update_box.text = "Error"

        if result["type"] == "final" or result["type"] == "error":
            set_is_running(False)
            text_input.model.set_value("")

async def main(query):
    async_gen = await asyncio.to_thread(run_agent, query)
    await update_output_box(output_box, async_gen)

def on_submit():
    global text_input, update_box, output_box
    set_is_running(True)
    update_box.text = "Thinking..."
    output_box.text = ""
    query = text_input.model.get_value_as_string()
    task = asyncio.run_coroutine_threadsafe(main(query), loop)
    running_tasks.append(task)

def on_stop():
    """
    Kill any running async tasks and threads
    """
    set_is_running(False)

def on_clear():
    output_box.text = ""
    update_box.text = ""
    text_input.model.set_value("")

def on_clear_memory():
    on_clear()
    agent.clear_chat()
    update_box.text = "Agent memory cleared."


class RosaKitExtExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[rosa.kit.ext] rosa kit ext startup")

        global label, text_input, output_box, update_box
        set_is_running(True)


        self._window = ui.Window("ROSA", height=ui.Pixel(400), width=ui.Pixel(700), dockPreference=ui.DockPreference.LEFT_BOTTOM)
        # main_dockspace = ui.Workspace.get_window("Browser")
        # self._window.dock_in(main_dockspace, ui.DockPosition.RIGHT, 1)
        with self._window.frame:
            with ui.VStack(content_clipping=False, height=ui.Percent(100), width=ui.Percent(100)):
                with ui.VStack():
                    label = ui.Label("How can I help you with IsaacSim or Nova Carter?", height=ui.Pixel(30))
                    text_input = ui.StringField(height=ui.Pixel(30))

                with ui.HStack():
                    ui.Button("Submit", clicked_fn=on_submit, width=ui.Pixel(80), height=ui.Pixel(30), tooltip="Submit the query to the agent.")
                    ui.Button("Clear", clicked_fn=on_clear, width=ui.Pixel(80), height=ui.Pixel(30), tooltip="Clear the input box and output.")
                    ui.Button("Stop", clicked_fn=on_stop, width=ui.Pixel(80), height=ui.Pixel(30), tooltip="Stop the current task.")
                    ui.Button("Reset", clicked_fn=on_clear_memory, width=ui.Pixel(100), height=ui.Pixel(30), tooltip="Clear the agent memory and chat history.")

                with ui.VStack(height=ui.Percent(100)):
                    ui.Spacer(height=ui.Pixel(10))
                    update_box = ui.Label("", height=ui.Pixel(10), word_wrap=True, alignment=ui.Alignment.LEFT_TOP)
                    ui.Spacer(height=ui.Pixel(5))
                    scroll = ui.ScrollingFrame(height=ui.Percent(75), horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED, vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED)
                    scroll.vertical_scrollbar_policy = ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED
                    scroll.horizontal_scrollbar_policy = ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED

                ui.Spacer(height=ui.Pixel(200))
                output_box = ui.Label("", word_wrap=True, alignment=ui.Alignment.LEFT_TOP)
                scroll.add_child(output_box)
                timeline_tools.set_update_output_box(update_box)
                carter_tools.set_update_output_box(update_box)
                isaacsim_tools.set_update_output_box(update_box)

    def on_shutdown(self):
        print("[rosa.kit.ext] rosa kit ext shutdown")
