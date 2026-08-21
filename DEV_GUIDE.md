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

# Viz
```Powershell
$env:MPLBACKEND = "TkAgg"
python cpp_eyetracker\tests\utils\cam_calib\viz_calib_chain.py
```

# Close Defender
```Powershell
Add-MpPreference -ExclusionPath "D:\capture"
Add-MpPreference -ExclusionPath "E:\capture"
Add-MpPreference -ExclusionProcess "hdf5_multi_process_child.exe"
```
