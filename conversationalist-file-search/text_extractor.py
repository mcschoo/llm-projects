import os
from pypdf import PdfReader
from docx import Document
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def extract_text(file_path):
    """Extracts text content from supported file types."""
    _, extension = os.path.splitext(file_path)
    extension = extension.lower()
    text = ""
    try:
        if extension == ".pdf":
            reader = PdfReader(file_path)
            for page in reader.pages:
                text += page.extract_text() or ""
        elif extension == ".docx":
            doc = Document(file_path)
            for para in doc.paragraphs:
                text += para.text + "\n"
        elif extension in {".txt", ".py", ".md", ".csv"}: # Treat as plain text
             # Try common encodings
             encodings_to_try = ['utf-8', 'latin-1', 'cp1252']
             for enc in encodings_to_try:
                 try:
                     with open(file_path, 'r', encoding=enc) as f:
                         text = f.read()
                     break # Success
                 except UnicodeDecodeError:
                     continue # Try next encoding
                 except Exception as e:
                     logging.warning(f"Could not read {file_path} with {enc}: {e}")
                     continue
             if not text:
                 logging.warning(f"Could not decode {file_path} with any tried encoding.")

        # Add more handlers for other extensions (xlsx, pptx, etc.)
        else:
            logging.warning(f"Unsupported file type: {extension} for file {file_path}")
            return None

        return text.strip()

    except Exception as e:
        logging.error(f"Error extracting text from {file_path}: {e}")
        return None
        