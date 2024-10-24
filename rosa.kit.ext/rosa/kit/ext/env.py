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

import os


def set_env_variables():
    APIM_SUBSCRIPTION_KEY=""
    AZURE_TENANT_ID=""
    AZURE_CLIENT_ID=""
    AZURE_CLIENT_SECRET=""

    # Model configuration
    AZURE_ENDPOINT=""
    API_ENDPOINT=""

    AZURE_DEPLOYMENT_ID=""
    DEPLOYMENT_ID=""

    OPENAI_API_TYPE=""
    API_TYPE=""

    OPENAI_API_VERSION=""
    API_VERSION=""


    # Inference API configuration
    VISION_OPENAI_API_TYPE=""
    VISION_AZURE_OPENAI_ENDPOINT=""
    VISION_OPENAI_API_KEY=""
    VISION_OPENAI_API_VERSION=""
    VISION_CHAT_MODEL=""
    VISION_CHAT_DEPLOYMENT=""
    VISION_DEPLOYMENT_NAME=""
    VISION_AZURE_DEPLOYMENT=""

    os.environ["APIM_SUBSCRIPTION_KEY"] = APIM_SUBSCRIPTION_KEY
    os.environ["AZURE_TENANT_ID"] = AZURE_TENANT_ID
    os.environ["AZURE_CLIENT_ID"] = AZURE_CLIENT_ID
    os.environ["AZURE_CLIENT_SECRET"] = AZURE_CLIENT_SECRET
    os.environ["AZURE_ENDPOINT"] = AZURE_ENDPOINT
    os.environ["API_ENDPOINT"] = API_ENDPOINT
    os.environ["AZURE_DEPLOYMENT_ID"] = AZURE_DEPLOYMENT_ID
    os.environ["DEPLOYMENT_ID"] = DEPLOYMENT_ID
    os.environ["OPENAI_API_TYPE"] = OPENAI_API_TYPE
    os.environ["API_TYPE"] = API_TYPE
    os.environ["OPENAI_API_VERSION"] = OPENAI_API_VERSION
    os.environ["API_VERSION"] = API_VERSION
    os.environ["VISION_OPENAI_API_TYPE"] = VISION_OPENAI_API_TYPE
    os.environ["VISION_AZURE_OPENAI_ENDPOINT"] = VISION_AZURE_OPENAI_ENDPOINT
    os.environ["VISION_OPENAI_API_KEY"] = VISION_OPENAI_API_KEY
    os.environ["VISION_OPENAI_API_VERSION"] = VISION_OPENAI_API_VERSION
    os.environ["VISION_CHAT_MODEL"] = VISION_CHAT_MODEL
    os.environ["VISION_CHAT_DEPLOYMENT"] = VISION_CHAT_DEPLOYMENT
    os.environ["VISION_DEPLOYMENT_NAME"] = VISION_DEPLOYMENT_NAME
    os.environ["VISION_AZURE_DEPLOYMENT"] = VISION_AZURE_DEPLOYMENT