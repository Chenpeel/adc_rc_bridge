FROM python:3.11-slim-bookworm

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /app

RUN pip install --no-cache-dir \
    "smbus2>=0.4.3" \
    "websockets>=12.0"

COPY bridge.py /app/bridge.py
COPY config.example.json /app/config.example.json

CMD ["python", "/app/bridge.py", "--config", "/app/config.json"]
