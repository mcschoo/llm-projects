# requirements.txt
# torch
# torchvision
# torchaudio
# transformers
# accelerate
# bitsandbytes # Optional, for quantization
# sentence-transformers
# pypdf
# python-docx
# chromadb
# tqdm # For progress bars during indexing

# ----------------------------------------
# main.py
# ----------------------------------------
import logging
import config
from file_indexer import build_index
from llm_interface import LlamaInterface
from retriever import Retriever

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def format_prompt_for_llm(query, context_docs):
    """Formats the prompt for the LLM with retrieved context."""
    context = "\n\n".join(context_docs)
    prompt = f"""Based *only* on the following context, answer the user's question. If the context doesn't contain the answer, say you don't have enough information from the provided files.

Context:
---
{context}
---

User Question: {query}

Answer:"""
    return prompt

def main_chat_loop(llm, retriever):
    print("\n--- Conversational File Search ---")
    print("Type 'exit' to quit, 'index' to rebuild the index.")
    print("Ask to 'find files about X' or ask questions like 'what are the requirements for project Y?'")

    conversation_history = [] # Simple list to store turns

    while True:
        try:
            user_input = input("\nYou: ")
            if user_input.lower() == 'exit':
                break
            if user_input.lower() == 'index':
                build_index()
                # Re-initialize retriever after indexing
                retriever = Retriever()
                print("Indexing complete. Please restart conversation.")
                conversation_history = []
                continue
            if not user_input:
                continue

            # --- Simple Intent Detection ---
            is_file_search = False
            search_keywords = ["find file", "search for file", "show file", "look for file"]
            if any(keyword in user_input.lower() for keyword in search_keywords):
                is_file_search = True
                # Extract keywords (simple approach)
                # TODO: Improve keyword extraction
                query_term = user_input.lower()
                for keyword in search_keywords:
                     query_term = query_term.replace(keyword, "")
                query_term = query_term.strip()


            # --- Add conversation history to query (optional, basic) ---
            # query_with_history = "\n".join([f"{turn['role']}: {turn['content']}" for turn in conversation_history])
            # query_with_history += f"\nuser: {user_input}"
            # For simplicity now, we won't use history directly in retrieval, only for LLM context if needed

            if is_file_search:
                logging.info(f"Performing file path search for: '{query_term}'")
                results = retriever.search_file_paths(query_term)
                print("\nLLM Assistant:")
                if results:
                    print("Found potential files:")
                    for path in results:
                        print(f"- {path}")
                else:
                    print("Could not find any matching file paths.")
                # Add to history (optional)
                # conversation_history.append({"role": "user", "content": user_input})
                # conversation_history.append({"role": "assistant", "content": "\n".join(results)})

            else: # Assume Content QA
                logging.info(f"Performing content QA search for: '{user_input}'")
                context_docs, metadatas = retriever.search_content(user_input)

                if not context_docs:
                    print("\nLLM Assistant: I couldn't find relevant information in the indexed files to answer that.")
                    # Add to history (optional)
                    # conversation_history.append({"role": "user", "content": user_input})
                    # conversation_history.append({"role": "assistant", "content": "No relevant context found."})
                    continue

                print(f"\n--- Debug: Retrieved context from ---")
                unique_sources = {meta.get('source', 'Unknown') for meta in metadatas}
                for source in unique_sources:
                     print(f"- {source}")
                print("--- End Debug ---")


                llm_prompt = format_prompt_for_llm(user_input, context_docs)
                # print(f"\n--- Debug: Prompt sent to LLM ---\n{llm_prompt}\n--- End Debug ---") # Uncomment for debugging

                response = llm.generate_response(llm_prompt)
                print(f"\nLLM Assistant: {response}")

                # Add to history (optional)
                # conversation_history.append({"role": "user", "content": user_input})
                # conversation_history.append({"role": "assistant", "content": response})

        except EOFError:
            break
        except KeyboardInterrupt:
            break
        except Exception as e:
            logging.error(f"An error occurred in the chat loop: {e}", exc_info=True)
            print("An unexpected error occurred. Please check logs.")

    print("\nExiting chat.")


if __name__ == "__main__":
    # --- Initialization ---
    # 1. Build index if it doesn't exist or if forced
    #    For the first run, you MUST run the indexing.
    #    You could add a check here: if not os.path.exists(config.VECTOR_DB_PATH) or not os.path.exists(config.INDEX_PATH):
    #        print("Index not found. Running initial indexing...")
    #        build_index()
    #        print("Initial indexing complete.")
    #    Or just run it manually first via `python file_indexer.py` or by typing 'index' in the chat.
    print("Ensure the index is built before starting the chat.")
    print("Run the script and type 'index' first if needed.")


    # 2. Load LLM and Retriever
    llm_interface = LlamaInterface()
    retriever = Retriever()

    # Check if components loaded successfully
    if llm_interface.model and retriever.vector_db_collection:
        # 3. Start interactive loop
        main_chat_loop(llm_interface, retriever)
    else:
        logging.error("Failed to initialize LLM or Retriever. Cannot start chat loop.")
        print("Initialization failed. Please check logs and configuration.")