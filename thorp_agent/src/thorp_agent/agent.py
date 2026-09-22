"""
ROSA with Thorp's capabilities as its only tools and Thorp's prompts as its only system prompt.

ROSA always adds 55 ROS introspection tools of its own, and a system prompt ordering the model to
list nodes and topics before every action. Both are left out here by overriding the two methods
ROSA builds them in.
"""

from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from rosa import ROSA


class _Tools(object):
    """Stands in for ROSATools, of which ROSA only calls get_tools()"""

    def __init__(self, tools):
        self._tools = list(tools)

    def get_tools(self):
        return self._tools


class ThorpAgent(ROSA):
    def _get_tools(self, ros_version, packages, tools, blacklist):
        return _Tools(tools or [])

    def _get_prompts(self, robot_prompts=None):
        # the placeholder names are the keys ROSA's invoke fills in
        messages = [robot_prompts.as_message()] if robot_prompts else []
        return ChatPromptTemplate.from_messages(messages + [
            MessagesPlaceholder(variable_name="chat_history"),
            ("user", "{input}"),
            MessagesPlaceholder(variable_name="agent_scratchpad"),
        ])
