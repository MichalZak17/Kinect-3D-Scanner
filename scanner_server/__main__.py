"""Entry point: python -m scanner_server"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "14")

import uvicorn  # noqa: E402

uvicorn.run("scanner_server.app:app", host="0.0.0.0", port=8000)
