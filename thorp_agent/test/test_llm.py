"""
The line each model call prints: how long it took, its tokens, and the client's retries.

    cd ~/colcon_ws/thorp/src/thorp/thorp_agent && python3 -m pytest test/test_llm.py -v
"""

import logging
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

pytest.importorskip('langchain_core')

from langchain_core.messages import AIMessage  # noqa: E402
from langchain_core.outputs import ChatGeneration, LLMResult  # noqa: E402

from thorp_agent import llm  # noqa: E402


def answered(input_tokens, output_tokens):
    message = AIMessage(content='', usage_metadata={'input_tokens': input_tokens, 'output_tokens': output_tokens,
                                                    'total_tokens': input_tokens + output_tokens})
    return LLMResult(generations=[[ChatGeneration(message=message)]])


def test_a_call_line_says_its_time_and_tokens():
    lines = []
    timing = llm.Timing(lines.append)
    timing.on_chat_model_start({}, [[]])
    timing.on_llm_end(answered(12345, 310))
    assert len(lines) == 1 and lines[0].endswith(' s, 12345 tokens in, 310 out')


def test_the_client_retries_are_counted_per_call():
    lines = []
    timing = llm.Timing(lines.append)
    timing.on_chat_model_start({}, [[]])
    logging.getLogger('openai._base_client').info('Retrying request to %s in %f seconds', '/chat/completions', 0.4)
    timing.on_llm_end(answered(100, 10))
    timing.on_chat_model_start({}, [[]])
    timing.on_llm_end(answered(100, 10))
    assert lines[0].endswith(', retried 1 time') and 'retried' not in lines[1]


def test_a_failed_call_says_why():
    lines = []
    timing = llm.Timing(lines.append)
    timing.on_chat_model_start({}, [[]])
    timing.on_llm_error(TimeoutError('Request timed out.'))
    assert lines[0].endswith(' s, failed: TimeoutError: Request timed out.')
