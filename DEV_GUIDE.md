# Development Guide

## Prerequisites 🧑‍🎓

- CMake 3.15 or higher
- Microsoft Visual C++ compiler

## Packages and Libraries 📚
- opencv4
- ceres
- eigen
- pugi-xml
- yaml-cpp

## basic cpp grammar📕

[notion link (Please send me the request for access)](https://www.notion.so/2792c8d1a6f580e38dace0999f545050)

## project structure management 📂

[notion link (Please send me the request for access)](https://www.notion.so/2792c8d1a6f580859333f87b5bdeca73)

## basler test
- bitznet
- vs2022
- cmake 4.2.1
- vcpkg
- configs: CMakePresets, config.yaml

# Build
```cmd
cmake --preset vs2022-vcpkg
cmake --build build --config Release
```