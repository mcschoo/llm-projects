# ----------------------------------------
# llm_interface.py
# ----------------------------------------
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
import logging
import config

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class LlamaInterface:
    def __init__(self):
        self.model = None
        self.tokenizer = None
        self._load_model()

    def _load_model(self):
        logging.info(f"Loading LLM: {config.LLM_MODEL_ID}")

        quantization_config = None
        torch_dtype = torch.float16 # Default if not quantizing heavily

        if config.LOAD_IN_4BIT:
            logging.info("Using 4-bit quantization")
            quantization_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_compute_dtype=torch.bfloat16, # Or float16 if bfloat16 not supported
                bnb_4bit_use_double_quant=True,
            )
            # Override dtype if using 4-bit
            torch_dtype = torch.bfloat16 # Or float16
        elif config.LOAD_IN_8BIT:
             logging.info("Using 8-bit quantization")
             quantization_config = BitsAndBytesConfig(load_in_8bit=True)
             # 8-bit usually uses float16 compute


        try:
            self.tokenizer = AutoTokenizer.from_pretrained(config.LLM_MODEL_ID)
            self.model = AutoModelForCausalLM.from_pretrained(
                config.LLM_MODEL_ID,
                device_map="auto", # Let accelerate handle device placement
                torch_dtype=torch_dtype,
                quantization_config=quantization_config,
                # trust_remote_code=True # Sometimes needed depending on model version
            )
            logging.info("LLM loaded successfully.")
        except Exception as e:
            logging.error(f"Failed to load LLM: {e}")
            logging.error("Check model ID, GPU memory, CUDA setup, and Hugging Face token.")
            # Consider falling back to CPU or exiting if model load fails
            self.model = None
            self.tokenizer = None


    def generate_response(self, prompt, max_new_tokens=250):
        if not self.model or not self.tokenizer:
            logging.error("LLM not loaded. Cannot generate response.")
            return "Error: Model not available."

        logging.info("Generating LLM response...")
        # Llama 3 uses a specific chat template format
        # Example using the chat template (adjust if using base model)
        messages = [
            # You might add a system prompt here if desired
             {"role": "user", "content": prompt}
        ]
        # Apply the chat template
        prompt_formatted = self.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True # Important for generation
        )

        inputs = self.tokenizer(prompt_formatted, return_tensors="pt").to(config.DEVICE)

        # Generation parameters
        outputs = self.model.generate(
            **inputs,
            max_new_tokens=max_new_tokens,
            eos_token_id=self.tokenizer.eos_token_id,
            do_sample=True, # Sample for more natural responses
            temperature=0.6,
            top_p=0.9,
        )

        # Decode the generated tokens, skipping the prompt part
        response_text = self.tokenizer.decode(outputs[0][inputs['input_ids'].shape[1]:], skip_special_tokens=True)
        logging.info("LLM response generated.")
        return response_text.strip()
