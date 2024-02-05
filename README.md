# From Human Demonstrations to Robot Polishing

![Language grade: Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
[![Made withMATLAB](https://img.shields.io/badge/Made%20with-MATLAB-blue?style=for-the-badge&logo=data:image/png;base64,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)](https://www.mathworks.com/products/matlab.html)

[[<b> Youtube Video </b>](https://youtu.be/H2vTvLHJsqw)] 

submitted to ICDL 2024

## TODO
- [ ] Add videos
- [ ] Read reviews
- [ ] Work on 2nd derivate DS
- [ ] Update paper with reviewers comments

## Dependencies 
- for python on offline data
```
pip install scipy matplotlib
```
- for python with ROS to track hand in real-time
```
pip install scipy matplotlib rospkg
```
- for python with ROS + kinova Gen3
```
pip install scipy matplotlib rospkg
```
and install [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) and source it before running the project
```
source PATH/TO/CATKINWORKSPACE/WITH/ROS_KORTEX/devel/setup.bash
```

## Run
choose your option
```
python online|offline_polish_dataset|kinova.py
```

## License 
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Rights
Nuno Duarte



