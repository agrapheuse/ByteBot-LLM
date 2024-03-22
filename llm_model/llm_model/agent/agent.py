import os

from langchain import hub
from langchain.agents import AgentExecutor, create_openai_tools_agent
from langchain.chains.openai_functions import create_structured_output_runnable
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI

from .plan import Plan
from .retriever import Retriever

BEHAVIOR_FILE = "/tmp/behavior.txt"
if not os.path.exists(BEHAVIOR_FILE):
    with open(BEHAVIOR_FILE, "w") as file:
        file.write("")


class Agent:
    def __init__(self, tools, logger):
        self.tools = tools
        prompt = hub.pull("hwchase17/openai-tools-agent")
        agent = create_openai_tools_agent(
            ChatOpenAI(model="gpt-4-turbo-preview", temperature=0.7),
            tools,
            prompt,
        )
        self.agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=True)
        self.logger = logger
        self.planner = self._create_planner()
        self.retriever = self._create_retriever()
        # self.rag = self._create_retriever()
        if not os.path.exists("/tmp/conversation.txt"):
            with open("/tmp/conversation.txt", "w") as file:
                file.write("")
        self.cache = []

    def act(self, input_text):
        """
        Generate a plan, act on that plan and return the result.
        """
        if input_text in self.cache:
            self.logger.info(f"Input text {input_text} is already in cache.")
            return
        if len(self.cache) > 1:
            self.cache = []
        self.cache.append(input_text)

        self.logger.info(f"Agent is acting on input: {input_text}")
        context = self.retriever.retrieve(input_text)
        # Create a plan
        plan = self.planner.invoke(
            {"objective": input_text, "tools": self.tools, "context": context}
        )

        conversation_so_far = self.retrieve_conversation_so_far()
        self.logger.info(f"Conversation so far: {conversation_so_far}")
        general_prompt = f"""
You are an AI Robot, you will be asked questions that you have to answer to or perform actions.
You have access to a variety of tools. You can use these tools to accomplish tasks.
Max 30 words.
Always use the speech tool to communicate with the user. You are talking to a user.

CONVERSATION SO FAR:
{conversation_so_far}

Current Step:

"""
        self.logger.info(f"Agent has created plan: {plan}")
        for step in plan.steps:
            self.logger.info(f"Agent is executing step: {step}")
            result = self.agent_executor.invoke({"input": general_prompt + step})
            self.logger.info(f"Agent has finished step {step}")
        with open("/tmp/conversation.txt", "a") as file:
            file.write(f"User: {input_text}\nAgent: {result}\n")

    def act_from_plan(self, plan: list[str]):
        self.logger.info(f"Plan received: {plan}")
        plan = plan['plan']
        if plan in self.cache:
            self.logger.info(f"Plan {plan} is already in cache.")
            return
        if len(self.cache) > 1:
            self.cache = []
        self.cache.append(plan)
        with open(BEHAVIOR_FILE, "r") as file:
            behavior = file.read()
        behavior_prompt = f"""
        Here is a guideline from your owner:
        ====
        {behavior}
        ====
        """

        general_prompt = f"""
        You are a superhuman robot that has access to a variety of tools. You can use these tools to accomplish tasks.
        Max 30 words.
        The user has passed in a plan of the following steps:
        {plan}
        
        You are talking to a user. In each step, communicate how the bot is meant to behave.
        {behavior_prompt if behavior else ""}
        The current step is:
        
        """
        for step in plan:
            self.logger.info(f"Agent is executing step: {step}")
            result = self.agent_executor.invoke({"input": general_prompt + step})
            self.logger.info(f"Agent has finished step {step}")
        with open("/tmp/conversation.txt", "a") as file:
            file.write(f"User: {plan}\nAgent: {result}\n")

    def act_from_tool(self, tool: str):
        self.logger.info(f"Tool received: {tool}")
        prompt = f"""
        Use the tool {tool}
        """
        self.act_from_plan([prompt])

    def _create_planner(self):
        """
        This function takes in input text and returns a plan to follow.
        """
        planner_prompt = ChatPromptTemplate.from_template(
            """For the given objective, come up with a simple step by step plan. \
This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.


# Here is an overview of tools at your disposal:
{tools}
TIP: Sleep means going to the dock
# Here is the objective to plan for:
{objective}

# Here is some possible context:
{context}

TIP: The objective can be a bit mumbled since it is coming from a flawed speech to text system. If you don't understand it, ask for clarification.
IMPORTANT! There should be as little steps as possible. Usually 1-2 but can be more. The simpler the better.
"""
        )
        planner = create_structured_output_runnable(
            Plan, ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0), planner_prompt
        )
        return planner

    # def _create_replanner(self):
    #     """
    #     This function takes in input text and returns a plan to follow.
    #     """
    #     replanner_prompt = ChatPromptTemplate.from_template(
    #         """For the given objective, come up with a simple step by step plan. \
    # This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
    # The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.

    # Your objective was this:
    # {input}

    # Your original plan was this:
    # {plan}

    # You have currently done the follow steps:
    # {past_steps}

    # Update your plan accordingly. If no more steps are needed and you can return to the user, then respond with that. Otherwise, fill out the plan. Only add steps to the plan that still NEED to be done. Do not return previously done steps as part of the plan."""
    #     )
    #     replanner = create_openai_fn_runnable(
    #         [Plan, Response],
    #         ChatOpenAI(model="gpt-4-turbo-preview", temperature=0),
    #         replanner_prompt,
    #     )
    #     return replanner

    def retrieve_conversation_so_far(self):
        """
        Retrieve the conversation so far from the graph.
        """
        # read from tmp
        conversation_so_far = ""
        with open("/tmp/conversation.txt", "r") as file:
            conversation_so_far = file.read()

        if len(conversation_so_far) > 1000:
            conversation_so_far = conversation_so_far[-1000:]
        return conversation_so_far

    def _create_retriever(self):
        USER_FOLDER = os.path.expanduser("~")
        USER_FOLDER = os.path.expanduser("~")
        KNOWLEDGE_PATH = os.path.join(
            USER_FOLDER, "bytebot", "knowledge", "llm-context"
        )
        if not os.path.exists(KNOWLEDGE_PATH):
            os.makedirs(KNOWLEDGE_PATH)
            # Make a file with Asimov's laws
            with open(os.path.join(KNOWLEDGE_PATH, "laws.txt"), "w") as file:
                file.write(
                    "1. A robot may not injure a human being or, through inaction, allow a human being to come to harm.\n"
                    "2. A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.\n"
                    "3. A robot must protect its own existence as long as such protection does not conflict with the First or Second Law."
                )
            with open(os.path.join(KNOWLEDGE_PATH, "tb4project.txt"), "w") as file:
                file.write(
                    """
                    ByteBot is a team of 3 students, Filip Nowak, Elina van der Taelen and Noah Diderich.
                    We came together with a vision of making a turtlebot have a human-like conversation with a user.
                    We are using Python, OpenCV, ROS2, and GPT-4 to make this happen.

                    Our supervisors are Geert de Paepe and Levi Slap
                    """
                )
        retriever = Retriever(knowledge_folder=KNOWLEDGE_PATH)
        return retriever
