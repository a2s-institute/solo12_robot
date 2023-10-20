## Generating an URDF

In case of modifications to the `solo12.urdf.xacro` file it is necessary to regenerate the `solo12.urdf` file, this in order to avoid parsing at runtime (which is possible but slow) and speed up the launch process:

```
xacro solo12.urdf.xacro > solo12.urdf && check_urdf solo12.urdf
```
