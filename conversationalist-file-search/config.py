import os

# important paths
# !! IMPORTANT: Use raw strings (r"...") or double backslashes for Windows paths !!
DRIVE_ROOT = r"\\your_server\your_share\target_folder" 
INDEX_PATH = "file_index.json" 
VECTOR_DB_PATH = "vector_db" # ChromaDB persistent storage !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

# models
LLM_MODEL_ID = "meta-llama/Meta-Llama-3-8B" 
EMBEDDING_MODEL_ID = "sentence-transformers/all-MiniLM-L6-v2"
DEVICE = "cuda" # change to cpu for testing **********************************************

# indexing
SUPPORTED_EXTENSIONS = {".txt", ".py", ".md", ".pdf", ".docx", ".csv"} # ** CHANGE FOR ARX !!!!!!!!!!!!!!!!!!!
CHUNK_SIZE = 750 # Tokens/characters per chunk (tune this)
CHUNK_OVERLAP = 50 # Overlap between chunks

# retrieval - how much context to feed the model
NUM_RELEVANT_CHUNKS = 3 

# Quantization (requires bitsandbytes)
LOAD_IN_8BIT = False
LOAD_IN_4BIT = True # saves more VRAM, will be less important for ARX model
