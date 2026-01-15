# lecture3_examples
Examples of using Eigen, custom message files, and ros2 arguments.

Rotates a vector by a specified angle

run example using:

```bash
ros2 launch lecture3_example rotate_vector.launch.yaml <arg name>:=<value>
```

The launch arguments are:

| arg name | default | description |
|----------|---------|-------------|
| `x` | `0.0` | x comp. of vector |
| `y` | `0.0` | y comp. of vector |
| `theta` | `0.0` | angle to rotate vector by | 
