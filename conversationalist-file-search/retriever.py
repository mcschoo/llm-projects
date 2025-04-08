import json
import chromadb
from chromadb.utils import embedding_functions
import logging
import config

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class Retriever:
    def __init__(self):
        self.file_paths = self._load_file_index()
        self.vector_db_collection = self._load_vector_db()

    def _load_file_index(self):
        try:
            with open(config.INDEX_PATH, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            logging.error(f"File index {config.INDEX_PATH} not found. Run indexing first.")
            return []
        except Exception as e:
            logging.error(f"Error loading file index: {e}")
            return []

    def _load_vector_db(self):
        try:
            client = chromadb.PersistentClient(path=config.VECTOR_DB_PATH)
            # IMPORTANT: Use the same embedding function as during indexing
            sentence_transformer_ef = embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name=config.EMBEDDING_MODEL_ID, device=config.DEVICE
            )
            collection = client.get_collection(
                name="file_content", embedding_function=sentence_transformer_ef
            )
            logging.info("Vector DB collection loaded.")
            return collection
        except Exception as e:
            # Catch specific ChromaDB errors if needed (e.g., collection not found)
            logging.error(f"Error loading vector DB collection: {e}")
            logging.error("Ensure the vector DB exists and was created with the correct embedding function.")
            return None

    def search_file_paths(self, query_keywords):
        """Simple keyword search on file paths."""
        if not self.file_paths:
            return ["Error: File index not loaded."]

        results = []
        keywords = query_keywords.lower().split()
        if not keywords:
            return ["Please provide keywords to search for."]

        logging.info(f"Searching file paths for keywords: {keywords}")
        for path in self.file_paths:
            path_lower = path.lower()
            # Simple check if all keywords appear in the path
            if all(keyword in path_lower for keyword in keywords):
                results.append(path)

        return results if results else ["No matching file paths found."]

    def search_content(self, query_text, k=config.NUM_RELEVANT_CHUNKS):
        """Semantic search in the vector database."""
        if not self.vector_db_collection:
             return [], [] # Return empty lists if DB not loaded

        logging.info(f"Performing semantic search for: '{query_text}'")
        try:
            results = self.vector_db_collection.query(
                query_texts=[query_text],
                n_results=k,
                include=['documents', 'metadatas'] # Get text and source info
            )

            retrieved_docs = results.get('documents', [[]])[0]
            retrieved_metadatas = results.get('metadatas', [[]])[0]

            logging.info(f"Retrieved {len(retrieved_docs)} relevant chunks.")
            return retrieved_docs, retrieved_metadatas # Return text and metadata

        except Exception as e:
            logging.error(f"Error querying vector database: {e}")
            return [], []

