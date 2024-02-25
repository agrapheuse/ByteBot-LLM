from ragatouille import RAGPretrainedModel
import os
from langchain_community.document_loaders import UnstructuredFileLoader
from langchain_community.embeddings import HuggingFaceBgeEmbeddings
from langchain_community.vectorstores import FAISS
from langchain.retrievers import ContextualCompressionRetriever

USER_FOLDER = os.path.expanduser("~")
KNOWLEDGE_PATH = os.path.join(USER_FOLDER, "knowledge")
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
        return self.rag.get_relevant_documents(query)

    def _create_rag(self, knowledge_folder: str):
        RAG = RAGPretrainedModel.from_pretrained("colbert-ir/colbertv2.0")
        files = os.listdir(knowledge_folder)
        files = [os.path.join(knowledge_folder, file) for file in files]
        loader = UnstructuredFileLoader(files, mode="elements")
        texts = loader.load()

        model_name = "BAAI/bge-small-en"
        model_kwargs = {"device": "cpu"}
        encode_kwargs = {"normalize_embeddings": True}
        hf = HuggingFaceBgeEmbeddings(
            model_name=model_name,
            model_kwargs=model_kwargs,
            encode_kwargs=encode_kwargs,
        )
        retriever = FAISS.from_documents(texts, hf).as_retriever(
            search_kwargs={"k": 20}
        )
        compression_retriever = ContextualCompressionRetriever(
            base_compressor=RAG.as_langchain_document_compressor(),
            base_retriever=retriever,
        )
        return compression_retriever
