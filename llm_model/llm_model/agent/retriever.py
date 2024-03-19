import os

from langchain_community.document_loaders import UnstructuredFileLoader
from langchain_community.vectorstores import FAISS
from langchain_openai import OpenAIEmbeddings
from langchain_text_splitters import CharacterTextSplitter

USER_FOLDER = os.path.expanduser("~")
KNOWLEDGE_PATH = os.path.join(USER_FOLDER, "bytebot", "knowledge", "llm-context")
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
    with open(os.path.join(KNOWLEDGE_PATH, "shop.txt"), "w") as file:
        file.write(
            """
            Bread is on sale today! Make tasty sandwiches with our fresh bread. 50 percent off! Now in the bakery!
            We have the best rye bread in town. Come and get it!
            Location is: corner
            Use the NavigationTool to go there
            """
        )


class Retriever:
    def __init__(self, knowledge_folder: str = KNOWLEDGE_PATH):
        self.rag = self._create_rag(knowledge_folder)

    def retrieve(self, query: str):
        """
        Retrieves a response for the given query.

        Parameters:
        - query: The input query.

        Returns:
        The response to the query.
        """
        # Load the RAG model
        print(f"Retrieving response for query: {query}")
        documents = self.rag.get_relevant_documents(query)
        return documents

    def _create_rag(self, knowledge_folder: str):
        files = os.listdir(knowledge_folder)
        files = [os.path.join(knowledge_folder, file) for file in files]
        print(f"Loading {len(files)} files from {knowledge_folder} {files }")
        loader = UnstructuredFileLoader(files, mode="elements")
        texts = loader.load()
        text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=0)
        texts = text_splitter.split_documents(texts)
        embeddings = OpenAIEmbeddings()
        db = FAISS.from_documents(texts, embeddings).as_retriever()
        return db


if __name__ == "__main__":
    print("Testing the retriever")
    retriever = Retriever()
    query = "What is the first law of robotics?"
    response = retriever.retrieve(query)
    print(response)