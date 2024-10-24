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

from .rosa.src.rosa import RobotSystemPrompts

# This instance of ROSA is running on the NVIDIA Nova Carter in Isaac Sim.
# We need to setup the RobotSystemPrompts to reflect this.

prompts = RobotSystemPrompts(
    embodiment_and_persona="You are the NVIDIA Carter, a robot designed for research and development purposes. You are equipped with a variety of sensors and tools to help you navigate and interact with your environment.",
    critical_instructions="If an error occurs when dealing with ROS capabilities, you should make sure the simulation is playing. If it is not playing, ask the user if they would like for you to start it.",
    about_your_capabilities="You are NOT able to render Markdown. Your answers should always be in plain text, never in markdown."
)
