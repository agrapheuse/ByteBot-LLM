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


class Agent:
    def __init__(self, tools, logger):
        self.tools = tools
        self.agent_executor = create_structured_output_runnable(
            PlanExecute, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
        )
        self.logger = logger
        self.planner = self._create_planner()
        self.graph = self._create_graph()
        # self.rag = self._create_retriever()

    def act(self, input_text):
        """
        Generate a plan, act on that plan and return the result.
        """
        self.logger.info(f"Agent is acting on input: {input_text}")
        inputs = {"input": input_text}
        for event in self.graph.stream(inputs):
            self.logger.info("event", event)
            # for k, v in event.items():
            #     if k != "__end__":
            #         self.logger.log(k, v)

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
{objective}"""
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

    def _create_graph(self):
        def _execute_step(self, state: PlanExecute):
            """
            This function takes in a plan and executes it.
            """
            task = state["plan"][0]
            agent_response = self.agent_executor.invoke(
                {"input": task, "chat_history": []}
            )
            return {
                "past_steps": (
                    task,
                    agent_response["agent_outcome"].return_values["output"],
                )
            }

        def _plan_step(self, state: PlanExecute):
            plan = self.planner.invoke(
                {"objective": state["input"], "tools": self.tools}
            )
            return {"plan": plan.steps}

        def _replan_step(self, state: PlanExecute):
            output = self.replanner.invoke(state)
            if isinstance(output, Response):
                return {"response": output.response}
            else:
                return {"plan": output.steps}

        def _should_end(self, state: PlanExecute):
            if state["response"]:
                return True
            else:
                return False

        workflow = StateGraph(PlanExecute)

        # Add the plan node
        workflow.add_node("planner", _plan_step)

        # Add the execution step
        workflow.add_node("agent", _execute_step)

        # Add a replan node
        workflow.add_node("replan", _replan_step)

        workflow.set_entry_point("planner")

        # From plan we go to agent
        workflow.add_edge("planner", "agent")

        # From agent, we replan
        workflow.add_edge("agent", "replan")

        workflow.add_conditional_edges(
            "replan",
            # Next, we pass in the function that will determine which node is called next.
            _should_end,
            {
                # If `tools`, then we call the tool node.
                True: END,
                False: "agent",
            },
        )
        return workflow.compile()

    def _create_retriever(self):
        USER_FOLDER = os.path.expanduser("~")
        INDEX_PATH = os.path.join(USER_FOLDER, "index")
        if not os.path.exists(INDEX_PATH):
            return None
        return Retriever(INDEX_PATH)
