"""
Which model the agent talks to. One function, so swapping provider is one place.

OpenRouter by default: one key and one base_url reach every model, so comparing them is a change of string rather than
of code. ROSA documents OpenAI, Azure and Ollama but takes any langchain chat model, so the others work the same way.
"""

import logging
import os
import time

from langchain_core.callbacks import BaseCallbackHandler

OPENROUTER_BASE_URL = 'https://openrouter.ai/api/v1'

# Seconds a model call may take before the client retries it. A step takes seconds; one taking much longer has most
# likely stalled at the provider, and a retry comes back sooner than waiting it out
TIMEOUT = 30.0

# The client libraries retry calls themselves, out of langchain's sight, and say so on these loggers
RETRY_LOGGERS = ('openai._base_client', 'anthropic._base_client')

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


def make(provider=None, model=None, temperature=0.0, trace=None):
    """The chat model; with trace, a function taking a line, it reports each call's time and tokens there"""
    provider = (provider or os.getenv('THORP_AGENT_PROVIDER') or 'openrouter').lower()
    model = model or os.getenv('THORP_AGENT_MODEL') or DEFAULTS.get(provider)
    callbacks = [Timing(trace)] if trace else None

    if provider == 'openrouter':
        # No max_tokens: on a reasoning model it also caps the hidden reasoning, and a step that thinks past it comes
        # back with the tool call truncated or missing
        from langchain_openai import ChatOpenAI
        return ChatOpenAI(
            model=model,
            api_key=_env('OPENROUTER_API_KEY', provider),
            base_url=OPENROUTER_BASE_URL,
            temperature=_temperature(model, temperature),
            timeout=TIMEOUT,
            default_headers={'X-Title': 'thorp_agent'},
            callbacks=callbacks,
            **_reasoning(model))

    if provider == 'anthropic':
        # console.anthropic.com; billed separately from a Claude.ai subscription
        from langchain_anthropic import ChatAnthropic
        return ChatAnthropic(model=model, temperature=temperature, timeout=TIMEOUT,
                             api_key=_env('ANTHROPIC_API_KEY', provider), callbacks=callbacks)

    if provider == 'openai':
        from langchain_openai import ChatOpenAI
        return ChatOpenAI(model=model, temperature=_temperature(model, temperature), timeout=TIMEOUT,
                          api_key=_env('OPENAI_API_KEY', provider), callbacks=callbacks)

    if provider == 'ollama':
        # nothing leaves the machine; tool calling wants a model trained for it
        from langchain_ollama import ChatOllama
        return ChatOllama(model=model, temperature=temperature, num_ctx=8192, callbacks=callbacks)

    raise ValueError('unknown provider {!r}: try {}'.format(provider, ', '.join(DEFAULTS)))


class Timing(BaseCallbackHandler):
    """A line per model call: how long it took, its tokens in and out, and how many times the client retried it"""

    def __init__(self, trace):
        self.trace = trace
        self.started = time.monotonic()
        self.retries = 0
        counter = _RetryCounter(self)
        for name in RETRY_LOGGERS:
            logger = logging.getLogger(name)
            logger.setLevel(min(logger.getEffectiveLevel(), logging.INFO))
            logger.addHandler(counter)

    def on_chat_model_start(self, serialized, messages, **kwargs):
        self.started, self.retries = time.monotonic(), 0

    def on_llm_start(self, serialized, prompts, **kwargs):
        self.started, self.retries = time.monotonic(), 0

    def on_llm_end(self, response, **kwargs):
        message = getattr(response.generations[0][0], 'message', None) if response.generations else None
        usage = getattr(message, 'usage_metadata', None) or {}
        line = '{:.1f} s'.format(time.monotonic() - self.started)
        if usage:
            line += ', {} tokens in, {} out'.format(usage.get('input_tokens'), usage.get('output_tokens'))
        self.trace(line + self._retried())

    def on_llm_error(self, error, **kwargs):
        self.trace('{:.1f} s, failed: {}: {}{}'.format(time.monotonic() - self.started, type(error).__name__, error,
                                                       self._retried()))

    def _retried(self):
        return ', retried {} time{}'.format(self.retries, '' if self.retries == 1 else 's') if self.retries else ''


class _RetryCounter(logging.Handler):
    def __init__(self, timing):
        super().__init__(logging.INFO)
        self.timing = timing

    def emit(self, record):
        if record.getMessage().startswith('Retrying request'):
            self.timing.retries += 1


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
