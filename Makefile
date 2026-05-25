.PHONY: up down backend frontend install demo test

# Docker shortcuts
up:
	docker-compose up -d

down:
	docker-compose down

# Local dev - backend
backend:
	cd backend && uvicorn src.server:app --host 0.0.0.0 --port 8000 --reload

# Local dev - frontend
frontend:
	cd frontend && npm run dev

# Install all dependencies
install:
	pip install -r requirements.txt
	cd frontend && npm install

# Generate demo show and launch backend
demo:
	@echo "Generating demo show and launching backend..."
	DEMO_MODE=1 cd backend && uvicorn src.server:app --host 0.0.0.0 --port 8000 --reload &
	@sleep 2
	curl -s -X POST http://localhost:8000/api/generate_demo | python3 -m json.tool
	@echo "Backend running at http://localhost:8000 with demo show loaded."

# Run tests
test:
	pytest backend/tests/ -v --tb=short
