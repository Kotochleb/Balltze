# Balltze Description

Description of robot Balltze with simple docker tools for generating and previewing URDF model using docker.



## Generate urdf
Generate urdf file from xacro and save as *balltze.urdf*
```bash
cd examples/generate_urdf
docker-compose build
docker-compose up
docker-compose logs --no-color --no-log-prefix | tail -n +2 > ../../balltze_description/urdf/balltze.urdf
```

## Preview model in RViz
### Nvidia GPU
Requires [`nvidia-docker`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
```bash
cd examples/rviz_nvidia_preview
xhost local:root
docker-compose up --build
```

### Intel and AMD GPU
```bash
cd examples/rviz_preview
xhost local:root
docker-compose up --build
```

### Windows


```bash
cd examples/rviz_windows_preview
docker-compose up --build
```
This section is based on article by [Jack Kawell](https://jack-kawell.com/2019/09/11/setting-up-ros-in-windows-through-docker/).
For this setup install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) and use this configuration:
<!-- TODO add own image -->
![](https://jackkawell.files.wordpress.com/2019/08/image-5.png)


## Defining own robot
You can define most of your robot's parameters in *balltze_description/config/props.yaml*.


#### Units:
- `mass` - *[kg]*
- `length` | `width` | `thickness` | `origin` - *[m]*
- `limit.lower` | `limit.upper` - *[rad]*
- `velocity` - *[m/s]*
- `effort` - *[N/m]*
- `inertia` - *[kg/m^2]*

#### Different origins:
- `joint.origin` - point relative to parent link where joint is attached.
- `link.origin` - point where center of mesh is defined relative to joint origin

#### Arrays in *props.yaml*
In *config.yaml* many values are defined as arrays. Each value in array defines parameters for each leg. Values in array are in given order:
| **index** |   **leg**   |
| :-------: | :---------: |
|    `0`    | front right |
|    `1`    | front left  |
|    `2`    | rear  right |
|    `3`    | rear  left  |

#### Meshes:
Meshes have to be defined in *balltze_description/meshes* folder. Suggested format is *STL*. `inertia` has to be relative to the same origin as *STL* origin is.