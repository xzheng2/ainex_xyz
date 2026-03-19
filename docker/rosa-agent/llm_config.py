"""
LLM configuration for the Ainex ROSA agent.

Reads LLM_PROVIDER from environment (or .env file) and returns the appropriate
LangChain chat model. Supported providers:
  - ollama   (default, no API key, requires Ollama on host)
  - openai   (requires OPENAI_API_KEY)
  - azure    (requires AZURE_OPENAI_* env vars)
"""

import os
from dotenv import load_dotenv

load_dotenv()  # load from .env if present


def get_llm():
    """Return a configured LangChain LLM based on LLM_PROVIDER env var."""
    provider = os.environ.get("LLM_PROVIDER", "ollama").lower()

    if provider == "ollama":
        from langchain_ollama import ChatOllama
        base_url = os.environ.get("OLLAMA_BASE_URL", "http://host.docker.internal:11434")
        model = os.environ.get("OLLAMA_MODEL", "llama3.2")
        return ChatOllama(base_url=base_url, model=model, temperature=0)

    elif provider == "openai":
        from langchain_openai import ChatOpenAI
        api_key = os.environ.get("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY is required when LLM_PROVIDER=openai")
        model = os.environ.get("OPENAI_MODEL", "gpt-4o")
        return ChatOpenAI(api_key=api_key, model=model, temperature=0)

    elif provider == "azure":
        from langchain_openai import AzureChatOpenAI
        return AzureChatOpenAI(
            azure_endpoint=os.environ["AZURE_OPENAI_ENDPOINT"],
            api_key=os.environ["AZURE_OPENAI_API_KEY"],
            azure_deployment=os.environ["AZURE_OPENAI_DEPLOYMENT_NAME"],
            api_version=os.environ.get("AZURE_OPENAI_API_VERSION", "2024-05-01-preview"),
            temperature=0,
        )

    else:
        raise ValueError(
            f"Unknown LLM_PROVIDER='{provider}'. "
            "Supported values: ollama, openai, azure"
        )
