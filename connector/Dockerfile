FROM python:3.11-slim AS base

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

WORKDIR /app

COPY pyproject.toml uv.lock ./

# ── Test stage: install everything, run tests, then discard ──
FROM base AS test

RUN uv sync --frozen
COPY src/ src/
COPY tests/ tests/
RUN uv run pytest tests/ -v

# ── Production stage: only prod deps, no test files ──
FROM base AS production

RUN uv sync --frozen --no-dev --no-install-project
COPY src/ src/
RUN uv sync --frozen --no-dev

EXPOSE 8080

CMD ["uv", "run", "uvicorn", "connector.main:app", "--host", "0.0.0.0", "--port", "8080", "--log-level", "info"]
