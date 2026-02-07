# ROS2 Foundation - Teaching Materials


[![Open in Dev Container](https://img.shields.io/badge/Dev_Container-Open-blue?logo=visualstudiocode)](https://vscode.dev/redirect?url=vscode://ms-vscode-remote.remote-containers/cloneInVolume?url=https://github.com/Amynwabu/ROS2-Foundation)
ROS2 teaching materials- includes code examples, packages, and tutorials organized by lesson.

## 📋 Prerequisites

- **ROS2 Humble** installed on Ubuntu 22.04
- **VS Code** or preferred IDE
- Basic Python programming knowledge
- Familiarity with Linux terminal commands

## 🚀 Getting Started

## Export Workspace

```bash
### Download the zip file                                 
wget "https://github.com/Amynwabu/ROS2-Fundation/raw/main/ros2_ws.zip"

### Create and extract into workspace

source /opt/ros/humble/setup.bash       # ensure Humble env
mkdir -p ~/ros2_ws/src
unzip ros2_ws.zip -d ~/ros2_ws/src/     # extract contents to src/
cd ~/ros2_ws
```


## 📚 Repository Structure
```
The repository is organized by lessons, with each folder containing:
- Working code files
- Documentation and instructions
- Examples you can run directly
```

```
ROS2-Foundation/
├── README.md
├── lesson01_environment_workspace/
│   ├── README.md
│   ├── check_env.sh
│   └── create_workspace_commands.txt
├── lesson02_packages_nodes/
│   ├── README.md
│   ├── create_package_commands.txt
│   └── test2_py_pkg/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       └── test2_py_pkg/
│           ├── __init__.py
│           ├── mynode.py
│           ├── waiter.py
│           └── chef.py
├── lesson03_topics_publish_subscribe/
│   ├── README.md
│   └── topic_creator.py
└── lesson04_build_and_run/
    └── README.md
```



## 🔧 Building and Running Examples

To build the example package:

```bash
cd ~/ros2_ws
colcon build --packages-select test2_py_pkg
source install/setup.bash
```

To run example nodes:

```bash
ros2 run test2_py_pkg my_first_node
ros2 run test2_py_pkg create_topic
```

## 🐛 Troubleshooting

If you encounter errors:

1. **Source your workspace**: `source ~/ros2_ws/install/setup.bash`
2. **Rebuild the package**: `colcon build --packages-select test2_py_pkg`
3. **Check ROS2 installation**: Run the environment check script in lesson01
4. **Verify Python syntax**: Use VS Code linter or run `python3 -m py_compile <filename>.py`

## 📝 Contributing

Found a typo or bug? Please open an issue or submit a pull request!

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Created for MSc Applied AI students at Cranfield University
- Based on ROS2 Humble documentation and best practices

## 📧 Contact

For questions or feedback, please open an issue in this repository.
