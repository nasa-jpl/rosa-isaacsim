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

import dotenv
import os
from azure.identity import DefaultAzureCredential
from langchain_openai import AzureChatOpenAI

def get_token():
    credential = DefaultAzureCredential()
    token = credential.get_token("https://management.azure.com/.default")
    return token.token


def get_llm(streaming: bool = False):
    """A helper function to get the LLM instance."""
    dotenv.load_dotenv(dotenv.find_dotenv())

    APIM_SUBSCRIPTION_KEY = get_env_variable("APIM_SUBSCRIPTION_KEY")
    default_headers = {}
    if APIM_SUBSCRIPTION_KEY is not None:
        # only set this if the APIM API requires a subscription...
        default_headers["Ocp-Apim-Subscription-Key"] = APIM_SUBSCRIPTION_KEY

    llm = AzureChatOpenAI(
        azure_deployment=get_env_variable("DEPLOYMENT_ID"),
        azure_ad_token_provider=get_token,
        openai_api_type="azure_ad",
        api_version=get_env_variable("API_VERSION"),
        azure_endpoint=get_env_variable("API_ENDPOINT"),
        default_headers=default_headers,
        streaming=streaming,
    )
    return llm


def get_env_variable(var_name: str) -> str:
    """
    Retrieves the value of the specified environment variable.

    Args:
        var_name (str): The name of the environment variable to retrieve.

    Returns:
        str: The value of the environment variable.

    Raises:
        ValueError: If the environment variable is not set.

    This function provides a consistent and safe way to retrieve environment variables.
    By using this function, we ensure that all required environment variables are present
    before proceeding with any operations. If a variable is not set, the function will
    raise a ValueError, making it easier to debug configuration issues.
    """
    value = os.getenv(var_name)
    if value is None:
        raise ValueError(f"Environment variable {var_name} is not set.")
    return value