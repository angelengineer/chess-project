import ollama
from gtts import gTTS
import os

# 1. Generar texto con Ollama
print("🤖 Generando respuesta con Llama3...")
response = ollama.chat(
    model="llama3:8b",
    messages=[
        {
            "role": "user",
            "content": "cuentame un chiste"
        }
    ]
)

texto = response["message"]["content"]
print(f"\n💬 LLM dice: {texto}\n")

# 2. Convertir texto a audio (Guardar MP3)
print("🔊 Convirtiendo texto a voz...")
tts = gTTS(text=texto, lang='ja')
archivo_audio = "respuesta.mp3"
tts.save(archivo_audio)

# 3. Reproducir audio usando el sistema (mpg123)
print("▶️ Reproduciendo...")
# El flag -q es para 'quiet' (que no llene la pantalla de texto técnico)
os.system(f"mpg123 -q {archivo_audio}") 

# Opcional: Borrar el archivo después de reproducir
os.remove(archivo_audio)