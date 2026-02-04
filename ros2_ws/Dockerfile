FROM ros:jazzy

# Evita preguntas interactivas
ENV DEBIAN_FRONTEND=noninteractive

# Dependencias del sistema
RUN apt update && apt install -y \
    python3-pygame \
    python3-pip \
    stockfish \
    && rm -rf /var/lib/apt/lists/*

# Dependencias Python
RUN pip3 install --break-system-packages \
    requests \
    python-chess \
    ollama

# Workspace
WORKDIR /ros2_ws

# Source ROS siempre
SHELL ["/bin/bash", "-c"]
