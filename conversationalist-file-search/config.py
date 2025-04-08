# ----------------------------------------
# config.py
# ----------------------------------------
import os

# --- Paths ---
# !! IMPORTANT: Use raw strings (r"...") or double backslashes for Windows paths !!
DRIVE_ROOT = r"\\your_server\your_share\target_folder" # Root directory to scan
INDEX_PATH = "file_index.json" # Simple index for file paths
VECTOR_DB_PATH = "vector_db" # Directory for ChromaDB persistent storage

# --- Models ---
LLM_MODEL_ID = "meta-llama/Meta-Llama-3-8B" # Or instruct version if preferred
EMBEDDING_MODEL_ID = "sentence-transformers/all-MiniLM-L6-v2"
DEVICE = "cuda" # Or "cpu" if no GPU, but VERY slow

# --- Indexing ---
SUPPORTED_EXTENSIONS = {".txt", ".py", ".md", ".pdf", ".docx", ".csv"} # Add more as needed
CHUNK_SIZE = 750 # Tokens/characters per chunk (tune this)
CHUNK_OVERLAP = 50 # Overlap between chunks

# --- Retrieval ---
NUM_RELEVANT_CHUNKS = 3 # How many chunks to feed to LLM as context

# --- LLM Generation ---
# Quantization (optional, requires bitsandbytes)
LOAD_IN_8BIT = False
LOAD_IN_4BIT = True # More aggressive, saves more VRAM
