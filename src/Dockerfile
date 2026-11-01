FROM python:3.11-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libegl1 libxrender1 libxcursor1 libsm6 libice6 libxext6 \
    && rm -rf /var/lib/apt/lists/*
COPY src/requirements-cad.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt
WORKDIR /workspace
