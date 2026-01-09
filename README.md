# ROS2 MSc Applied AI - Teaching Materials

ROS2 teaching materials for MSc Applied AI students - includes code examples, packages, and tutorials organized by lesson.

## 📋 Prerequisites

- **ROS2 Humble** installed on Ubuntu 22.04
- **VS Code** or preferred IDE
- Basic Python programming knowledge
- Familiarity with Linux terminal commands

## 🚀 Getting Started

### Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/Amynwabu/ros2-msci-applied-ai.git
cd ~/ros2_ws
colcon build --packages-select test2_py_pkg
source install/setup.bash
```

## 📚 Repository Structure

The repository is organized by lessons, with each folder containing:
- Working code files
- Documentation and instructions
- Examples you can run directly

```
ros2-msci-applied-ai/
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

## 🎯 How to Use This Repository

### For Students

1. **Clone the repository** instead of copying code from slides
2. **Open files in VS Code** for coding tasks
3. **Follow the README** in each lesson folder for step-by-step instructions
4. **Run the examples** to see working code in action
5. **Modify and experiment** with the code to deepen your understanding

### For Instructors

When showing code on slides, reference the GitHub location:
- "Code in GitHub: `lesson02_packages_nodes/test2_py_pkg/mynode.py`"
- Encourage students to open files from the cloned repo in VS Code
- All code is pre-tested and syntax-verified

## 📖 Lesson Overview

### Lesson 01: Environment & Workspace
- Check ROS2 installation
- Create and configure workspace
- Understand workspace structure

### Lesson 02: Packages & Nodes
- Create ROS2 Python packages
- Build and understand nodes
- Package configuration and setup

### Lesson 03: Topics - Publish & Subscribe
- Create publishers and subscribers
- Understand topic communication
- Message passing between nodes

### Lesson 04: Build & Run
- Build packages with colcon
- Source workspace
- Run nodes and debug

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
