# LLM Projects

This is a workspace for most of the public llm work I do

## Project Summaries

#### Sound Device Project
    - Call to Google's Gemini API to process user commands that will be sent down to ROS
    - Commands can be sent via command line or through a connected audio device that's capable of taking in input audio

#### LLM ROS Interactions Project
    - 3-tier node pipeline to send user commands to ROS2 (Humble)
    - Top level command processing node takes in a user command, and sends it to a text summarizer
    - Text summarizer adds user context, and sends the updated version of the user command to an altered version of the above sound device project (Gemini)
    - Gemini returns a JSON structure representing the interpreted user command, and will send that command to a simulated robot in Gazebo

#### Conversationalist File Search Model
    - RAG-based system for conversational file search & Q&A on Windows shares
    - Extracts text (PDF, DOCX, etc.), chunks, embeds (Sentence Transformers), stores in ChromaDB
    - Uses semantic search (ChromaDB) and keyword search for user queries
    - Feeds retrieved context + query to Llama 3-8B (via Transformers) for grounded responses