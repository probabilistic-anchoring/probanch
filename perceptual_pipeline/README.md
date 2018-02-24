If you are using OpenCV3, you might have modify line 54 from:

```
set(EXPORTED_DEPENDENCIES OpenCL)
```
to
```
set(EXPORTED_DEPENDENCIES OpenCL)
add_definitions( -fexceptions )
```
to `ai_kinect2/kinect2_registration/CMakeLists.txt`