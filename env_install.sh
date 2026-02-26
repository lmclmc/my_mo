#!/bin/bash

conda create -n ros2 python=3.13.11 -y
source $(conda info --base)/etc/profile.d/conda.sh
conda activate ros2
pip install -r requirements.txt