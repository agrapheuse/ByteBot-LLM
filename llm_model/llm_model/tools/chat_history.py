from langchain_core.messages import HumanMessage


def read_chat_history():
    history = [
        HumanMessage(
            content="Can you quack like a duck with every message you sen?"
        ),
    ]
    return history
