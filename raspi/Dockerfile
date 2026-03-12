FROM python:3.11-slim-bookworm

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /app

# 使用 uv + uv.lock 做依赖安装，保证版本可复现
RUN pip install --no-cache-dir "uv"

COPY pyproject.toml uv.lock /app/
RUN uv sync --frozen

COPY bridge.py /app/bridge.py
COPY config.example.json /app/config.example.json

CMD ["uv", "run", "python", "/app/bridge.py", "--config", "/app/config.json"]
