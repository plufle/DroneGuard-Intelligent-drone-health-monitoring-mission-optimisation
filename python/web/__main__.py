"""python -m web  ->  the planner on http://127.0.0.1:8000"""

import uvicorn

if __name__ == "__main__":
    print("DroneGuard planner: open http://127.0.0.1:8000  (Ctrl+C to stop)")
    uvicorn.run("web.app:app", host="127.0.0.1", port=8000)
