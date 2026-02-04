import requests

response = requests.post(
    "http://localhost:11434/api/generate",
    json={
        "model": "mistral",
        "prompt": "Suggest a good chess move in algebraic notation"
    }
)

print(response.json()["response"])
