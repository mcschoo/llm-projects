# Conversationalist File Search LLM 
    - running through windows cmd
    - after activating the venv, stay in the venv

## Setup Virtual Environment
    cd conversationalist-file-search
    python -m venv venv
    venv\Scripts\activate
    - install specific pytorch version (only if you need for CUDA, or some other reason)
    pip install -r requirements.txt

## Configure config.py **IMPORTANT
    - Open config.py in a text editor.
    - Crucially: Set DRIVE_ROOT to the correct full path of the Windows shared drive folder you want to index (use raw strings r"\\server\share..." or double backslashes \\ ).
    - Verify LLM_MODEL_ID and EMBEDDING_MODEL_ID.
    - Set DEVICE to "cuda" if using GPU, or "cpu" if testing without GPU.
    - Adjust LOAD_IN_4BIT / LOAD_IN_8BIT based on your VRAM and desired performance/quality trade-off (set one to True if desired, requires bitsandbytes).
    - Review SUPPORTED_EXTENSIONS, CHUNK_SIZE, etc.

## Set-up Hugging Face
    - huggingface-cli login
    
## Before first run
    - python file_indexer.py

## Running the LLM
    - python main.py

### Reminders
    - GPU vs CPU: If DEVICE = "cpu", the LLM part will be extremely slow or won't run unless you've implemented the mocking or API call alternatives. Indexing will also be slower. 
    - Watch your memory usage and errors