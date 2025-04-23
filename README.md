# cart_control

# control

*humans should focus on bigger problems*

## Setup

\`\`\`bash
git clone git@github.com:anysphere/control
\`\`\`

\`\`\`bash
./init.sh
\`\`\`

## Folder structure

**The most important folders are:**

1. \`vscode\`: this is our fork of vscode, as a submodule.
2. \`milvus\`: this is where our Rust server code lives.
3. \`schema\`: this is our Protobuf definitions for communication between the client and the server.

Each of the above folders should contain fairly comprehensive README files; please read them. If something is missing, or not working, please add it to the README!

Some less important folders:

1. \`release\`: this is a collection of scripts and guides for releasing various things.
2. \`infra\`: infrastructure definitions for the on-prem deployment.
3. \`third_party\`: where we keep our vendored third party dependencies.

## Miscellaneous things that may or may not be useful

##### Where to find rust-proto definitions

They are in a file called \`aiserver.v1.rs\`. It might not be clear where that file is. Run \`rg --files --no-ignore bazel-out | rg aiserver.v1.rs\` to find the file.

## Releasing

Within \`vscode/\`:

- Bump the version
- Then:

\`\`\`
git checkout build-todesktop
git merge main
git push origin build-todesktop
\`\`\`

- Wait for 14 minutes for gulp and ~30 minutes for todesktop
- Go to todesktop.com, test the build locally and hit release
`


# Joystick Teleoperation for VESC Differential Drive

This ROS 2 setup allows you to teleoperate a differential drive robot using a joystick (e.g., Xbox or Logitech controller). The joystick commands are interpreted and translated into `Char` messages, which are then sent to a VESC-based differential drive controller over serial ports.

## 🧩 Package Components

### 1. Joystick Teleop Node
**Node Name**: `joystick_teleop`  
**Topic Published**: `cmd_vel` (`std_msgs/Char`)  
**Topic Subscribed**: `joy` (`sensor_msgs/Joy`)  

### 2. VESC Differential Drive Node
**Node Name**: `vesc_diff_drv`  
**Topic Subscribed**: `cmd_vel` (`std_msgs/Char`)  
**Serial Ports**: `/dev/ttyACM0` (Right), `/dev/ttyACM1` (Left)  
**Control Modes**: `duty`, `rpm`, `current` (default: `current`)  

## 🎮 How It Works

### Joystick Node
- Maps joystick axes to movement commands:
  - Forward: `e`
  - Backward: `c`
  - Left: `a`
  - Right: `d`
  - Stop: `q` (via button 0)
- Publishes these commands as ASCII `Char` messages.

### VESC Node
- Receives keypress commands (`e`, `c`, `a`, `d`, `q`)
- Maps them to VESC motor current changes
- Sends control packets via serial to VESC controllers for each wheel

## 🚀 How to Launch

### Step-by-Step Run Sequence
1. **Give permission to access serial ports**:
   ```bash
   sudo chmod 777 /dev/ttyACM0
   sudo chmod 777 /dev/ttyACM1
   ```

2. **Run `joy_node`**:
   ```bash
   ros2 run joy joy_node
   ```

3. **Run the Joystick Teleop Node**:
   ```bash
   ros2 run <your_package_name> joystick_teleop
   ```

4. **Run the VESC Diff Drive Node**:
   ```bash
   ros2 run <your_package_name> vesc_diff_drv
   ```

> 📝 Ensure `/dev/ttyACM0` and `/dev/ttyACM1` are correctly mapped to the VESCs and accessible.

## ⚙️ Parameters
You can override parameters using the ROS 2 CLI or launch file:
```bash
ros2 run <your_package_name> vesc_diff_drv --ros-args \
  -p right_serial_port:=/dev/ttyACM0 \
  -p left_serial_port:=/dev/ttyACM1 \
  -p control_mode:=current
```

## 🧪 Example Launch File
Here's a minimal launch file to run both nodes:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='your_package_name',
            executable='joystick_teleop',
            name='joystick_teleop',
            output='screen'
        ),
        Node(
            package='your_package_name',
            executable='vesc_diff_drv',
            name='vesc_diff_drv',
            output='screen',
            parameters=[
                {"right_serial_port": "/dev/ttyACM0"},
                {"left_serial_port": "/dev/ttyACM1"},
                {"control_mode": "current"},
            ]
        )
    ])
```

## 📌 Notes
- Ensure your user has permission to access serial devices (`/dev/ttyACM*`). You may need to add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```
- Reboot or log out and back in after changing group permissions.

## 🧹 Troubleshooting

- **Joystick not detected?**
  - Check that `joy_node` is running and your joystick is recognized by the system.
  - Use `jstest` or `evtest` to verify raw joystick input.

- **No motor movement?**
  - Check if `cmd_vel` messages are being published.
  - Confirm VESC is connected and powered properly.
  - Use `dmesg | grep tty` to verify USB serial device names.

- **Serial errors or timeouts?**
  - Verify correct baud rate and port in parameters.
  - Ensure no other process is using the serial port.

---

With this setup, you can smoothly control your differential drive robot via joystick using ROS 2!

