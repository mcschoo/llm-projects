import os
import json
import chromadb
from chromadb.utils import embedding_functions
from sentence_transformers import SentenceTransformer 
from tqdm import tqdm # Progress bar for fun
import logging

import config
import text_extractor

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def chunk_text(text, size=config.CHUNK_SIZE, overlap=config.CHUNK_OVERLAP):
    # Basic splitting by characters - could alter to token splitting (?)
    chunks = []
    start = 0
    while start < len(text):
        end = start + size
        chunks.append(text[start:end])
        start += size - overlap
        if start >= len(text):
             break # Avoid infinite loop on pure overlap
    return chunks

# scans drive, extracts text, chunks, embeds, and stores in an idx
def build_index():
    logging.info("Starting file indexing process...")
    file_paths = []
    all_chunks = []
    all_metadata = [] # store file path with each chunk
    doc_ids = [] # give each chunk an id we can reference later ************

    logging.info(f"Scanning directory: {config.DRIVE_ROOT}")
    file_scan_list = [] # tqdm so I can see progress
    for root, _, files in os.walk(config.DRIVE_ROOT):
        for file in files:
             file_scan_list.append(os.path.join(root, file))

    logging.info(f"Found {len(file_scan_list)} potential files. Processing supported types...")

    chunk_id_counter = 0
    for file_path in tqdm(file_scan_list, desc="Processing files"):
        relative_path = os.path.relpath(file_path, config.DRIVE_ROOT)
        _, extension = os.path.splitext(file_path)

        if extension.lower() in config.SUPPORTED_EXTENSIONS:
            file_paths.append(relative_path)
            logging.debug(f"Extracting text from: {relative_path}")
            text = text_extractor.extract_text(file_path)

            if text:
                logging.debug(f"Chunking text for: {relative_path}")
                chunks = chunk_text(text)
                for i, chunk in enumerate(chunks):
                    all_chunks.append(chunk)
                    all_metadata.append({"source": relative_path, "chunk_num": i})
                    doc_ids.append(f"{relative_path}_{i}")
                    chunk_id_counter += 1
            else:
                 logging.warning(f"No text extracted from: {relative_path}")
        else:
             logging.debug(f"Skipping unsupported file: {relative_path}")


    logging.info(f"Extracted {len(all_chunks)} text chunks from {len(file_paths)} files.")

    # store the file path index for reference later
    logging.info(f"Saving file path index to {config.INDEX_PATH}")
    try:
        with open(config.INDEX_PATH, 'w', encoding='utf-8') as f:
            json.dump(file_paths, f, indent=4)
    except Exception as e:
        logging.error(f"Failed to save file path index: {e}")
        return # end if no saving

    if not all_chunks:
        logging.warning("No text chunks were generated. Vector DB will be empty.")
        return

    # create our vector db, which we'll use to store and ref chunked data
    logging.info("Setting up vector database...")
    try:
        # Chroma embedding function to embedded, matching our model
        sentence_transformer_ef = embedding_functions.SentenceTransformerEmbeddingFunction(
            model_name=config.EMBEDDING_MODEL_ID, device=config.DEVICE
        )

        # important data ---> persistent storage 
        client = chromadb.PersistentClient(path=config.VECTOR_DB_PATH)

        # IMPORTANT: iff the collection exists with a different embedding function, this will error !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        collection = client.get_or_create_collection(
            name="file_content",
            embedding_function=sentence_transformer_ef,
            metadata={"hnsw:space": "cosine"}
        )  # Consider deleting the DB dir if changing embedding models.
        logging.info(f"Vector DB Collection 'file_content' ready.")

    except Exception as e:
        logging.error(f"Failed to initialize ChromaDB: {e}")
        logging.error("Ensure ChromaDB is installed and dependencies are met.")
        return

    # embed + add to vector db now that they're instantiated, chunk at a time
    logging.info("Embedding chunks and adding to vector database - don't panic if this runs for a long time...")
    batch_size = 64 # NEED TO ADJUST THIS VALUE BEFORE MOVING TO ARX HARDWARE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    num_batches = (len(all_chunks) + batch_size - 1) // batch_size

    try:
        for i in tqdm(range(num_batches), desc="Embedding Batches"):
            start_idx = i * batch_size
            end_idx = min((i + 1) * batch_size, len(all_chunks))

            batch_chunks = all_chunks[start_idx:end_idx]
            batch_metadata = all_metadata[start_idx:end_idx]
            batch_ids = doc_ids[start_idx:end_idx]

            if not batch_chunks: # Should not happen with correct batch logic, but safety check
                continue

            # Embeddings are handled by ChromaDB when adding if using its EF
            collection.add(
                documents=batch_chunks,
                metadatas=batch_metadata,
                ids=batch_ids
            )
        logging.info("Finished adding documents to vector database.")

    except Exception as e:
         logging.error(f"Error during embedding or adding to ChromaDB: {e}")
         logging.error("Check model loading, GPU memory, and ChromaDB status.")


    logging.info("Indexing process completed.")
