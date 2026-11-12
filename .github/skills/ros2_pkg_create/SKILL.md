---
name: ros2_pkg_create
description: Create a new C++20 ROS 2 package in src/<name> (package.xml, CMakeLists.txt, include/<name>.hpp node, src/main.cpp), publish it as its own GitHub repo on master and register it as a git submodule of ifc. Use when the user asks to create a new ROS module/package. Takes one argument - the package name.
---

# ros2_pkg_create

Argument: `name` – the package name (snake_case, e.g. `pose_logger`). If it is missing, ask for it with `ask_user`.

Derived values:
- `{{name}}` – the argument as given (package, executable, namespace and repo name).
- `{{Class}}` – `{{name}}` in PascalCase (`pose_logger` → `PoseLogger`).
- `{{maintainer}}` – output of `git config user.name`.
- `{{email}}` – output of `git config user.email`.

All commands run from the workspace root `/home/wgajda/dev/ifc`.

## 1. Preconditions

Stop and report if any check fails:
- `{{name}}` matches `^[a-z][a-z0-9_]*$`.
- `git config user.name` and `git config user.email` are both set.
- `src/{{name}}` does not exist and `{{name}}` is not already in `.gitmodules`.
- `.git/modules/src/{{name}}` does not exist (left over from an earlier attempt; `git submodule add` would reuse it). Ask the user before deleting it.

## 2. Create the package

Create the directories first, then the files:

```bash
mkdir -p src/{{name}}/include src/{{name}}/src
```

`src/{{name}}/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
    <name>{{name}}</name>
    <version>0.0.0</version>
    <description>{{name}} package for the IFC workspace.</description>
    <maintainer email="{{email}}">{{maintainer}}</maintainer>
    <license>TODO</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
    <depend>ifc_msgs</depend>
    <depend>rclcpp</depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

`src/{{name}}/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20)
project({{name}})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ifc_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ifc_msgs::ifc_msgs__rosidl_typesupport_cpp
    rclcpp::rclcpp
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ifc_msgs rclcpp)

ament_package()
```

`src/{{name}}/include/{{name}}.hpp`:

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace {{name}}
{

class {{Class}} : public rclcpp::Node
{
public:
    {{Class}}() : rclcpp::Node("{{name}}")
    {
    }
};

}
```

`src/{{name}}/src/main.cpp`:

```cpp
#include <{{name}}.hpp>

#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<{{name}}::{{Class}}>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

Replace every `{{...}}` placeholder (in paths too) with the derived values.

## 3. Add to the launch

The `ifc` package (`src/ifc`, part of the `ifc` repo, not a submodule) starts every node. Skip an edit if `{{name}}` is already there.

`src/ifc/launch/launch.py` – insert this `Node` so the `Node` entries stay in alphabetical order of `package`:

```python
        Node(
            package='{{name}}',
            executable='{{name}}',
            parameters=[params_file],
            output='screen',
        ),
```

`src/ifc/package.xml` – insert `<exec_depend>{{name}}</exec_depend>` so the `exec_depend` entries stay in alphabetical order.

## 4. Verify the build

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-up-to {{name}} ifc
python3 -c "import ast; ast.parse(open('src/ifc/launch/launch.py').read())"
```

It must build without warnings. Fix and rebuild before continuing.

## 5. Initialise the package repository

```bash
cd src/{{name}}
git init -b master
git add -A
git commit -m "auto init repo"
```

## 6. Publish

The user creates an empty repository on GitHub (no README, license or .gitignore). Tell them the commit hash, then ask with `ask_user` for the SSH URL in the form `git@github.com:OWNER/REPO.git` (suggest `git@github.com:in-flight-calibration/{{name}}.git`). Re-ask if it does not match `^git@github\.com:[^/]+/[^/]+\.git$`.

```bash
cd src/{{name}}
git remote add origin URL
git push -u origin master
git ls-remote origin refs/heads/master
```

The push is correct only when `git ls-remote` prints the same hash as `git rev-parse HEAD`. If the push fails (repo missing, not empty, no access), report the error, let the user fix it or give another URL (`git remote set-url origin URL`) and retry. Never continue to step 7 before the push is verified.

## 7. Replace with a submodule

```bash
rm -rf src/{{name}}
git submodule add URL src/{{name}}
git -C src/{{name}} rev-parse --abbrev-ref HEAD
git -C src/{{name}} rev-parse HEAD
git status --short .gitmodules src/{{name}} src/ifc
```

Check that the submodule is on `master` at the pushed hash and that `.gitmodules` and `src/{{name}}` are staged.

## 8. Finish

- Do **not** commit in `ifc`; the user commits the staged `.gitmodules`, the submodule pointer and the `src/ifc` launch changes.
- Do not edit `.github/copilot-instructions.md`.
- Report: package path, class name, remote URL, pushed hash, launch entry added.
