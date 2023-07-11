create virtual environment (e.g., in home folder)
  python3 -m venv dextgen

activate venv
  python3 -m venv dextgen

install wheel and rospkg
  pip install wheel
  pip install rospkg

install from requirements.txt
  cd dextgen
  python3 -m pip install -r requirements.txt

install dextgen
  pip install .