In Settings->Apps->Apps and Features
  Uninstall Ubuntu

In PowerShell as admin:
Install WSL with Ubuntu 20.04: https://learn.microsoft.com/en-us/windows/wsl/install
  wsl --install -d Ubuntu-20.04

In Ubuntu:
Update apt-get
  sudo apt-get update

Install some dependencies:
  sudo apt-get -y install libxcursor-dev patchelf openmpi-bin python3-pip

Install Mujoco:
  wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz && mkdir ~/.mujoco && mv mujoco210-linux-x86_64.tar.gz ~/.mujoco
  cd ~/.mujoco && tar -xf mujoco210-linux-x86_64.tar.gz
  echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/<USER>/.mujoco/mujoco210/bin' >> ~/.bashrc
  echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia' >> ~/.bashrc
  source ~/.bashrc
  pip install mujoco-py

Check Mujoco 1:
  cd ~/.mujoco/mujoco210/bin
  ./simulate ../model/humanoid.xml

Install miniconda: https://varhowto.com/install-miniconda-ubuntu-20-04/
  wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
  chmod +x Miniconda3-latest-Linux-x86_64.sh
  ./Miniconda3-latest-Linux-x86_64.sh

Close terminal then reopen
  conda config --set auto_activate_base false

Clone: https://github.com/amacati/dextgen
  cd ~
  git clone https://github.com/amacati/dextgen

Setup Dextgen:
  cd ~/dextgen
  conda env create -f environment.yaml
  conda activate dextgen
  pip install .
  conda install -c conda-forge glew
  conda install -c conda-forge mesalib
  conda install -c menpo glfw3
  sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so
  echo 'export CPATH=$CONDA_PREFIX/include' >> ~/.bashrc
  source ~/.bashrc

Check Mujoco 2:
  cd ~/dextgen
  python3 mujoco_test.py

Check DextGen:
  mpirun -n 4 python main.py --env FlatPJCube-v0
  python3 optim/test.py