"""
Which model the agent talks to. One function, so swapping provider is one place.

OpenRouter by default: one key and one base_url reach every model, so comparing them is a change of string rather than
of code. ROSA documents OpenAI, Azure and Ollama but takes any langchain chat model, so the others work the same way.
"""

import os

OPENROUTER_BASE_URL = 'https://openrouter.ai/api/v1'

# How hard a reasoning model thinks before each answer, which is most of the wait on a step: minimal, low, medium or
# high, or empty to leave the model its own default. Lower is faster, and too low starts getting tool arguments wrong,
# which costs a retry and the time with it.
REASONING_EFFORT = 'low'

DEFAULTS = {
    'openrouter': 'openai/gpt-5-mini',
    'anthropic': 'claude-sonnet-5',
    'openai': 'gpt-5-mini',
    'ollama': 'llama3.1:70b',
}


def make(provider=None, model=None, temperature=0.0):
    provider = (provider or os.getenv('THORP_AGENT_PROVIDER') or 'openrouter').lower()
    model = model or os.getenv('THORP_AGENT_MODEL') or DEFAULTS.get(provider)

    if provider == 'openrouter':
        # No max_tokens: on a reasoning model it also caps the hidden reasoning, and a step that thinks past it comes
        # back with the tool call truncated or missing
        from langchain_openai import ChatOpenAI
        return ChatOpenAI(
            model=model,
            api_key=_env('OPENROUTER_API_KEY', provider),
            base_url=OPENROUTER_BASE_URL,
            temperature=_temperature(model, temperature),
            timeout=120,
            default_headers={'X-Title': 'thorp_agent'},
            **_reasoning(model))

    if provider == 'anthropic':
        # console.anthropic.com; billed separately from a Claude.ai subscription
        from langchain_anthropic import ChatAnthropic
        return ChatAnthropic(model=model, temperature=temperature, timeout=120,
                             api_key=_env('ANTHROPIC_API_KEY', provider))

    if provider == 'openai':
        from langchain_openai import ChatOpenAI
        return ChatOpenAI(model=model, temperature=_temperature(model, temperature), timeout=120,
                          api_key=_env('OPENAI_API_KEY', provider))

    if provider == 'ollama':
        # nothing leaves the machine; tool calling wants a model trained for it
        from langchain_ollama import ChatOllama
        return ChatOllama(model=model, temperature=temperature, num_ctx=8192)

    raise ValueError('unknown provider {!r}: try {}'.format(provider, ', '.join(DEFAULTS)))


def _reasoning(model):
    """
    OpenRouter's reasoning effort, as extra_body rather than langchain's own `reasoning` field: that one switches
    langchain to the Responses API, which is not what OpenRouter serves.
    """
    effort = os.getenv('THORP_AGENT_REASONING', REASONING_EFFORT)
    if not effort or not _thinks(model):
        return {}
    return {'extra_body': {'reasoning': {'effort': effort}}}


def _thinks(model):
    """Whether this is a reasoning model: it thinks before answering, and fixes its temperature"""
    name = model.split('/')[-1]
    return name.startswith(('gpt-5', 'o1', 'o3', 'o4')) and 'chat' not in name


def _temperature(model, temperature):
    """
    None for a reasoning model, which accepts only its default. langchain drops it itself for a bare "gpt-5..." name,
    but not behind OpenRouter's "openai/" prefix, nor for o3 and o4.
    """
    return None if _thinks(model) else temperature


def _env(variable, provider):
    value = os.getenv(variable)
    if not value:
        raise RuntimeError('{} is not set, so the {} provider has no credentials'.format(variable, provider))
    return value
