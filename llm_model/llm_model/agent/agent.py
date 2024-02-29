import os

from langchain.chains.openai_functions import (
    create_openai_fn_runnable,
    create_structured_output_runnable,
)
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI
from langgraph.graph import END, StateGraph
from .plan import Plan, PlanExecute, Response
from .retriever import Retriever
from langchain.agents import AgentExecutor, create_openai_tools_agent
from langchain import hub

class Agent:
    def __init__(self, tools, logger):
        self.tools = tools
        prompt = hub.pull("hwchase17/openai-tools-agent")
        agent = create_openai_tools_agent(
            ChatOpenAI(model="gpt-4-turbo-preview", temperature=0.7), tools, prompt
        )
        self.agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=True)
        self.logger = logger
        self.planner = self._create_planner()
        self.rag = self._create_retriever()
        if not os.path.exists("/tmp/conversation.txt"):
            with open("/tmp/conversation.txt", "w") as file:
                file.write("")

    def act(self, input_text):
        """
        Generate a plan, act on that plan and return the result.
        """

        self.logger.info(f"Agent is acting on input: {input_text}")
        from langchain_core.messages import AIMessage, HumanMessage

        # Create a plan
        plan = self.planner.invoke(
            {"objective": input_text, "tools": self.tools}
        )

        conversation_so_far = self.retrieve_conversation_so_far()
        context = self.rag.retrieve(input_text)
        general_prompt = f"""
You are an AI Robot, you will be asked questions that you have to answer to or perform actions.
Always use the speech tool to communicate with the user.

{conversation_so_far}

The user input can be mumbled because it froms from speech-to-text, do your best to understand it:
{input_text}

Here is some possible context that you can use to answer the question:
{context}

Current Step:

"""
        self.logger.info(f"Agent has created plan: {plan}")
        for step in plan.steps:
            self.logger.info(f"Agent is executing step: {step}")
            result = self.agent_executor.invoke({"input": general_prompt + step})
            self.logger.info(f"Agent has finished step {step}")
        with open("/tmp/conversation.txt", "a") as file:
            file.write(f"User: {input_text}\nAgent: {result}\n")

    def _create_planner(self):
        """
        This function takes in input text and returns a plan to follow.
        """
        planner_prompt = ChatPromptTemplate.from_template(
            """For the given objective, come up with a simple step by step plan. \
This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.

You are the brain of a robot. You can do actions such as moving the robot, speaking and navigating to waypoints. Make use of these and come up with creative ways to express yourself
Here is an overview of tools at your disposal:
{tools}

Here is the objective to plan for:
{objective}

IMPORTANT! There should be as little steps as possible. Max 3
"""
        )
        planner = create_structured_output_runnable(
            Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), planner_prompt
        )
        return planner

    def _create_replanner(self):
        """
        This function takes in input text and returns a plan to follow.
        """
        replanner_prompt = ChatPromptTemplate.from_template(
            """For the given objective, come up with a simple step by step plan. \
    This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
    The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.

    Your objective was this:
    {input}

    Your original plan was this:
    {plan}

    You have currently done the follow steps:
    {past_steps}

    Update your plan accordingly. If no more steps are needed and you can return to the user, then respond with that. Otherwise, fill out the plan. Only add steps to the plan that still NEED to be done. Do not return previously done steps as part of the plan."""
        )
        replanner = create_openai_fn_runnable(
            [Plan, Response],
            ChatOpenAI(model="gpt-4-turbo-preview", temperature=0),
            replanner_prompt,
        )
        return replanner
    
    def retrieve_conversation_so_far(self):
        """
        Retrieve the conversation so far from the graph.
        """
        # read from tmp
        conversation_so_far = ""
        with open("/tmp/conversation.txt", "r") as file:
            conversation_so_far = file.read()
        return conversation_so_far

    def _create_retriever(self):
        USER_FOLDER = os.path.expanduser("~")
        INDEX_PATH = os.path.join(USER_FOLDER, "index")
        retriever = Retriever(INDEX_PATH)
        return retriever
