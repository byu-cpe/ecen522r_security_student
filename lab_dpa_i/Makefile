IN_ENV = . .venv/bin/activate;

env: .venv/bin/activate

.venv/bin/activate:
	python3 -m venv .venv
	$(IN_ENV) pip install -r requirements.txt