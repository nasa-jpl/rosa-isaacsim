#  Copyright (c) 2024 Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import base64
import cv2
import numpy as np
import os
import rospy
import time
import threading
from langchain.agents import tool
from openai import AzureOpenAI
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Empty
from typing import Optional, Tuple, List
import subprocess


def apply_depth_filters(depth: np.ndarray) -> np.ndarray:
    """
    Apply filters to the depth image to remove noise and improve quality.
    """
    return depth


def get_image_from(topic, topic_type, timeout: float = 1.0) -> Optional[Image]:
    print(f"Waiting for image on topic {topic}...")
    try:
        img = rospy.wait_for_message(topic, topic_type, timeout=timeout)
        print(f"Received image on topic {topic}.")
        return img
    except rospy.ROSException:
        rospy.logerr(f"Timeout ({timeout}s) waiting for image on topic {topic}.")
        return None
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted by ROS shutdown while waiting for image.")
        return None


def get_camera_rgb_img(timeout: float = 5.0) -> Optional[Image]:
    """
    Get the RGB image from the camera.
    """
    topic = f"/rgb_left"
    return get_image_from(topic, Image, timeout=timeout)


def get_camera_depth_img(robot_name: str = 'carter', timeout: float = 1.0) -> Optional[Image]:
    """
    Get the depth image from the camera.
    """
    topic = f"/{robot_name}/camera_front/aligned_depth_to_color/image_raw"
    return get_image_from(topic, Image, timeout=timeout)


def display_images(imgs: List):
    """
    Display an image using OpenCV.
    """
    [cv2.imshow(f"Image {i}", img) for i, img in enumerate(imgs)]
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def as_base64_jpg(imgs: List[np.ndarray]) -> List[str]:
    """
    Convert an image to a base64 string.
    """
    return [base64.b64encode(cv2.imencode('.jpg', img)[1]).decode("utf-8") for img in imgs]


def as_base64_png(imgs: List[np.ndarray]) -> List[str]:
    """
    Convert an image to a base64 string.
    """
    return [base64.b64encode(cv2.imencode('.png', img)[1]).decode("utf-8") for img in imgs]


def get_rgb_and_depth_images(robot_name) -> Tuple[Optional[Image], Optional[Image]]:
    """
    Get RGB and depth images from the camera. Uses threading to get both images simultaneously.
    """
    rgb, depth = None, None

    def get_rgb():
        nonlocal rgb
        rgb = get_camera_rgb_img(robot_name)

    def get_depth():
        nonlocal depth
        depth = get_camera_depth_img(robot_name)

    threads = [threading.Thread(target=get_rgb), threading.Thread(target=get_depth)]
    [thread.start() for thread in threads]
    [thread.join() for thread in threads]

    return rgb, depth


def get_formatted_rgb(rotate: bool = True, timeout=4.0):
    """
    Get the RGB image from the camera and format it as a numpy array.
    """
    rgb = get_camera_rgb_img(timeout=timeout)

    if not rgb:
        return None

    rgb_img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
    if rotate:
        rgb_img = cv2.rotate(rgb_img, cv2.ROTATE_90_CLOCKWISE)
    return rgb_img


def combine_rgb_and_depth(rgb: Image, depth: Image, rotate_180: bool = False, debug: bool = False):
    """
    Combine the RGB and depth images into a single RGBA image.
    """
    # Convert to numpy arrays
    depth_img = np.frombuffer(depth.data, dtype=np.uint16).reshape(depth.height, depth.width)
    rgb_img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)

    # Rotate 180 degrees if necessary
    depth_img = cv2.rotate(depth_img, cv2.ROTATE_180) if rotate_180 else depth_img
    rgb_img = cv2.rotate(rgb_img, cv2.ROTATE_180) if rotate_180 else rgb_img

    # Remove the first 40 pixels on the left side of the depth image
    cropped_depth = depth_img[:, 45:]
    depth_img = cv2.resize(cropped_depth, (rgb.width, rgb.height), interpolation=cv2.INTER_LINEAR)

    # Normalize depth image and convert to 8-bit
    depth_img = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

    # Apply filters to the depth image
    depth_img = apply_depth_filters(depth_img)

    depth_img = cv2.applyColorMap(depth_img, cv2.COLORMAP_JET)

    # Combine color image and depth colormap
    combined_img = cv2.addWeighted(rgb_img, 0.7, depth_img, 0.3, 0)

    if debug:
        display_images([rgb_img, depth_img, combined_img])

    return rgb_img, depth_img, combined_img


def process_images(base64_imgs: List[str], prompts: Optional[List[dict]] = None):
    """
    Process the images and return a description of the scene.
    """
    import dotenv
    dotenv.load_dotenv()

    client = AzureOpenAI(
        azure_deployment=os.getenv("VISION_DEPLOYMENT_NAME"),
        api_version=os.getenv("VISION_OPENAI_API_VERSION"),
        azure_endpoint=os.getenv("VISION_AZURE_OPENAI_ENDPOINT"),
        api_key=os.getenv("VISION_OPENAI_API_KEY"),
    )

    messages = prompts or [{"role": "system", "content": "Please describe the scene in the image(s)."}]

    messages.extend([{
        "role": "user",
        "content": [
            {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{img}",
                    "detail": "high"
                }
            }
        ]
    } for img in base64_imgs])

    response = client.chat.completions.create(model="gpt-4o", messages=messages)
    result = response.choices[0].message.content
    return result


def get_combined_rgb_and_depth(robot_name: str, rotate_180: bool = False):
    """
    Get a combined RGB and depth image from the robot's camera.
    """
    rgb, depth = get_rgb_and_depth_images(robot_name)

    if not rgb:
        return {"error": "Could not get RGB image."}
    if not depth:
        return {"error": "Could not get depth image."}

    _, _, combined = combine_rgb_and_depth(rgb, depth, rotate_180=rotate_180, debug=False)
    return combined


def get_object_analysis_prompts(robot_name: str, objects_of_interest: List[str], instructions: Optional[str] = None):
    prompts = [
        {
            "role": "system",
            "content": f"You are a robot named '{robot_name}' developed by NASA Jet Propulsion Laboratory to analyze "
                       f"your operating environment and provide actionable information."
        },
        {
            "role": "system",
            "content": "You are equipped with a camera to interpret the scene around you. You must practice scientific "
                       "curiosity and objectivity in your analysis, which will be used to inform decisions and actions "
                       "taken by your human operators."
        },
        {
            "role": "system",
            "content": "Be thorough and accurate in your analysis. Consider what information would be most useful to "
                       "your operators and provide that information in a clear and concise manner."
        },
        {
            "role": "system",
            "content": "This is a one-shot task, so you must provide all relevant information in your initial response."
        }
    ]

    if instructions:
        prompts.append({
            "role": "system",
            "content": f"The following instructions will guide you in analyzing the scene: {instructions}."
        })

    prompts.append({
        "role": "system",
        "content": f"Please identify the following objects in the scene: {objects_of_interest}."
    })

    return prompts


@tool
def analyze_environment(robot_name: str, subjects: List[str], instructions: Optional[str] = None, raise_head: Optional[bool] = False):
    """
    Analyzes the environment around the robot and returns a description of the scene.
    Use this tool to analyze specific objects of interest in the camera feed.
    You should ALWAYS use this tool if the user asks for a specific description of the scene.

    :param robot_name: The name of the robot to use.
    :param subjects: The object(s) of interest to analyze. These should be as specific, yet nuanced, as possible. Include multiple objects if necessary.
    :param instructions: Instructions on how to perform the analysis.
    :param raise_head: Whether to raise the head module before capturing the image. Only set to True if the user requests it.

    :note: clear and concise instructions are important to guide the analysis. Common instructions may include:
    - "Count the number of object X in the scene."
    - "Estimate the distance to the object."
    - "Identify any obstacles in the path of object X."
    - "Describe the color, shape, and size of object X."
    - "Provide background information on object X."
    - etc.
    """

    rgb = get_camera_rgb_img()

    if not rgb:
        return {"error": "Could not get RGB image."}

    rgb_img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)

    # rgb_img = cv2.rotate(rgb_img, cv2.ROTATE_90_CLOCKWISE)

    imgs_base64 = as_base64_png([rgb_img])

    # Get prompts specific to the subjects
    prompts = get_object_analysis_prompts(
        robot_name=robot_name,
        objects_of_interest=subjects,
        instructions=instructions
    )

    response = process_images(imgs_base64, prompts=prompts)
    return response


@tool
def inspect_and_describe_camera_feed(raise_head: Optional[bool] = False):
    """
    Interprets the camera feed from the robot and returns a description of the scene.
    Use this tool to analyze the entire scene in the camera feed.

    :param raise_head: Whether to raise the head module before capturing the image. Only set to True if the user requests it.
    """
    rgb = get_camera_rgb_img()

    rgb_img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
    # rgb_img = cv2.rotate(rgb_img, cv2.ROTATE_90_CLOCKWISE)

    # convert to brg
    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)

    imgs_base64 = as_base64_png([rgb_img])

    response = process_images(imgs_base64, prompts=[{
        "role": "system",
        "content": scene_understanding_prompt
    }])
    # lower_head_after_image_capture()
    return response


scene_understanding_prompt = """
You are an AI designed to interpret scenes from a robot's camera and provide actionable information. 
You will be provided with an image of the scene. Return the results in JSON format.
---
Example:

{
  "description": "[Description of the scene]",
  "objects": [
    {
      "object": "<name>",
      "distance": "<estimated distance in meters>",
      "angle": "<estimated angle in degrees>",
      "obstacles": "<list of potential obstacles>"
    }
  ]
}
"""
