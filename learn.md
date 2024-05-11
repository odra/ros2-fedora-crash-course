# ROS2 Crash Course

The Robotics Operating System project, aka ROS, provides a fundation framework for robotics development, allowing
engineers to focus on the high level functionalities and dealing with all the common
low level details.

This course uses python based projects but any language could be used as long as it has
a RCL (ROS Client Libary) support.

Last but not least, ROS2 is the oficial successor of ROS, with the later marked tagged as EOL.

## Development Environment

* ROS Rolling: https://docs.ros.org/en/rolling/
* Fedora 39 Robotics SIG Container Image: https://gitlab.com/fedora/sigs/robotics/images/container-image-ros2
* Container Toolbox: https://containertoolbx.org/

NOTE: The fedora container images builds ROS2 from source since it's not an official supported platform.

You can build the ROS2 container image by running:

```
podman build -t localhost/ros2/toolbox:latest .
```

Now create and enter into your ros2 toolbox:

```
toolbox create --image localhost/ros2/toolbox:latest ros2
```

```
toolbox enter ros2
```

Set yourself as the owner of `/opt/ros`:

```
sudo chown $(whoami) -R /opt/ros
```

### Quick Test

ROS provided a talker/listener demo app that can be used to test if ROS2 is working.

Run each of the following commands in a terminal tab:

```
source /opt/ros/install/setup.bash
ros2 run demo_nodes_cpp listener
```

```
source /opt/ros/install/setup.bash
ros2 run demo_nodes_cpp talker
```

Logs from both apps should be shown as expected from an usual "pub/sub" demo app.

NOTE: you will need to source `/opt/ros/install/setup.bash` everytime you enter your toolbox or open a new tab
at the time of this writing.

## Writing ROS2 Packages

All ROS2 programs are defined as packages, so those can be easily built, re-used and distributed within
the ROS2 system.

### ROS2 Workspace

The first step is to create a new ROS2 workspace by:

* Create a new directory;
* Run `colcon build` within that directory.

Create a new workspace directory where you want, for example:

```
mkdir -p ~/Work/ros2-ws/src
```

Now cd into `~~/Work/ros2-ws` and run:

```
colcon build
```

Colcon will not build anything at this point but you will see that ir created the basic structure of your worspace:

```
$ tree .
.
├── build
│   └── COLCON_IGNORE
├── install
│   ├── COLCON_IGNORE
│   ├── _local_setup_util_ps1.py
│   ├── _local_setup_util_sh.py
│   ├── local_setup.bash
│   ├── local_setup.ps1
│   ├── local_setup.sh
│   ├── local_setup.zsh
│   ├── setup.bash
│   ├── setup.ps1
│   ├── setup.sh
│   └── setup.zsh
├── log
│   ├── COLCON_IGNORE
│   ├── build_2024-02-17_11-37-10
│   │   ├── events.log
│   │   └── logger_all.log
│   ├── latest -> latest_build
│   └── latest_build -> build_2024-02-17_11-37-10
└── src
```

### Python Package

Lets cd into our `src` workspace directory and create a new python package:

```
ros2 pkg create sample_py_pkg --build-type ament_python --dependencies rclpy
```

This will create a new pyhton package structure:

```
$ tree sample_py_pkg/
sample_py_pkg/
├── package.xml
├── resource
│   └── sample_py_pkg
├── sample_py_pkg
│   └── __init__.py
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

4 directories, 8 files
```

You can check that a "package.xml" file was created which has all package
metadata information, such as name, license, dependencies and so on.

You can build your package from the workspace directory, either by building all your packages
or just the one you created:

### C++ Package

somrthing


### Building

You can `colcon build...` commands to build your packages from your
ROS2 workspace directory:

```
# Build all packages in src
colcon build --packages-select

# filter what gets built
colcon build --packages-select sample_py_pkg
```

NOTE: Running `colcon build` without any options
will build all packages in `src`, which can take a while.

You can check your workspace folders, such as the `build` one, to see the generated build files from your command.

Installed programs will usually be located at `install/...`.

## ROS2 Nodes

Nodes are contained within ROS2 packages, so a package can contain N nodes.

Nodes are subprograms that do one thing really well.

Imagine you have a "camera package", this package could have the following nodes:

* Camera Status Management (turn on/off, allocate, retrieve status)
* Camera Driver (camera hardware integration)
* ...

Nodes from one package can communicate with nodes from another package, and such
communication is performned through messaging bus and topics, aka ROS2 Middleware.

This also means that nodes are language agnostic and communicate with each other
regardless of the used programming language.

There are several ROS2 middleware implentations which will be expanded a bit later.

To summarize, packages are a way to organize your ROS2 programs while nodes are
the actual programs that get run within your ROS2 robotics stack.

### Writing a Simple ROS2 Node

`RCL` stands for ROS Client Library, so you have RCL + $LANG libs such as `py`, `cpp`, `java`, `go` and so on.

ROS packages and nodes in the end are just an usual project that uses RCL of you programming language of choice.

You can create a python node just by adding a new .py file in your package,
such as `src/sample_py_pkg/sample_py_pkg/simple_py_node.py`:

```
import rclpy
from rclpy.node import Node as BaseNode


class Node(BaseNode):
    def __init__(self):
        self._name = 'sample_py'
        self._counter = 0
        super(Node, self).__init__(self._name)
        self.get_logger().info('Simple Python Node initialized')
        self.create_timer(0.5, self.callback)

    def callback(self):
        self._counter += 1
        self.get_logger().info(f'[{self._name}] Counter: {self._counter}')


def main(args=None):
    # start node
    rclpy.init(args=args)

    # instantiate node obj and log something
    node = Node()
    rclpy.spin(node) # keeps the node process alive

    # shutdown node
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

You also need to add your new node cli as a setup.py console script:

```
...
 entry_points={
        'console_scripts': [
            'simple_py_node = sample_py_pkg.simple_py_node:main',
        ],
    },
...
```

Now if you run `colcon build` again, you shall see your new node file in the `install` folder:

```
$ stat install/sample_py_pkg/lib/sample_py_pkg/simple_py_node 
  File: install/sample_py_pkg/lib/sample_py_pkg/simple_py_node
  Size: 990       	Blocks: 8          IO Block: 4096   regular file
Device: 253,1	Inode: 8407267     Links: 1
Access: (0755/-rwxr-xr-x)  Uid: ( 1000/lrossetti)   Gid: ( 1000/lrossetti)
Access: 2024-02-17 18:50:24.165944061 -0300
Modify: 2024-02-17 15:38:36.673272582 -0300
Change: 2024-02-17 15:38:36.673272582 -0300
 Birth: 2024-02-17 13:09:22.925644005 -0300
```

### Running a ROS2 Node

Running a node is quite simple, you just need to run `ros2 run $pkg_name $node_name`:

```
ros2 run sample_py_pkg simple_py_node
```

### Node Inspection

ROS2 provides some basic tooling to see and inspect your running nodes with the `node` subcommand: `ros2 node ....`.

First, run the sample node in one terminal tab/screen:

```
ros2 run sample_py_pkg simple_py_node
```

Now, in another tab/screen, you can run `ros2 node list` to get the list of running nodes:

```
$ ros2 node list
/sample_py
```

The node subcommand also provides an `info` subcommand to inspect some information about
a node (make sure to include the `/` from the node name):

```
$ ros2 node info /sample_py
/sample_py
  Subscribers:

  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /sample_py/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /sample_py/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /sample_py/get_parameters: rcl_interfaces/srv/GetParameters
    /sample_py/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /sample_py/list_parameters: rcl_interfaces/srv/ListParameters
    /sample_py/set_parameters: rcl_interfaces/srv/SetParameters
    /sample_py/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

#### Multiples Instances of a Node

Running the same node multiples times as it is will result in unexpected behavior since
ROS2 just uses the package and node name by default.

You will get both nodes running but the graph database won't be able to make
a distinction between then and only one of the nodes will be stored there.

You need to specify extra options to the `run` subcommand
to spawn multiple instances of the same node:

```sh
ros2 run sample_py_pkg simple_py_node --ros-args --remap __node:=sample_py2
```

Running `ros2 node list` will now list both nodes:

```
$ ros2 node list
/sample_py
/sample_py2
```

It's running the same node binary but with a different name.

### Node Debugging

You can use `qrt` to debug and see what's going on in your ROS2 stack locally.

`RQT` is a ROS2 QT application, it's in fact a ROS2 node!

You can run one or more nodes in your terminal, open a new one and run `rqt`. It will open a GUI qt window
which you can have a graphical representation of your nodes, the messaging bus and so on.

### The Turtlesim Node

ROS2 has a built-in simulation Node based on QT called `turtlesim`.

## ROS2 Messaging

* your usual topic based `publisher -> subscriber` pattern implementation
* messages are anonymous, which means that no auth and no way to know who sent which message
* a topic is always bound to a message type
* that's how node communicate with each other
* it's what allows nodes to be programming language agnostic
* nodes can have as many pubs or subs


`-r robot_news=my_news`

## ROS2 Services

It's your usual client -> service pattern where a server serves N clients.

Useful when you need to make node talk or connect to each other in a direct manner without
broadcasting messages to the whole system.

### Writing a ROS2 Server

```
ros2 service call /sum_two_ints example_interfaces/srv/AddTwoInts  '{"a": 3, "b": "5"}'
```

### Writing a ROS2 Client

```
import rclpy
from rclpy.node import Node as BaseNode
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)

    node = BaseNode('sum_two_ints_cli')
    client = node.create_client(AddTwoInts, 'sum_two_ints')
    
    while not client.wait_for_service(5.0):
        node.get_logger().warn('Waiting for server sum_two_ints')

    req = AddTwoInts.Request()
    req.a = 3
    req.b = 5

    # res = client.call(req)
    fn = client.call_async(req)
    node.get_logger().info('Waiting for response')
    rclpy.spin_until_future_complete(node, fn)

    try:
        res = fn.result()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')

    node.get_logger().info(f'Sum: {res.sum}')

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
```

### Debugging

```
ros2 service...
ros2 service list
ros2 service info
ros2 service type
ros2 service call
```

```
rqt
```

### Remap/Rename

```sh
--ros-args -r $svc_name:=new_name
```

### Turtlesim Services

```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleo_key
ros2 service list
....
```

```
$ ros2 service type /clear
std_srvs/srv/Empty
$ ros2 service call /clear std_srvs/srv/Empty
```

## ROS2 Interfaces

Interfaces are message definitions (types) to be used by messages.

### Messages (msg)

* defined in a package
* source code message structure generated by ros (compilation time)
* [Upstream sample interfaces](https://github.com/ros2/example_interfaces)
  *  Examples and to not be used in prod.
* [Upstream common interfaces](https://github.com/ros2/common_interfaces)

#### Defining

```
ros2 pkg create sample_interfaces

ros2 interface show sample_interfaces/msgs/HWState
```

#### Management

* List messages
* Check message schema

#### Testing

* Run client

### Services (srv)

* defined in a package
* functions that have their signature defintion
* use messages as input/output
* Code is generated during compilation time

#### Defining

#### Managament

#### Testing

### Workload Management

#### ROS2 Paramaters

* Name and data type
* Parametrize ROS2 nodes
* Declared in ROS2 node source code
* No global parameters
* it seems to be dynamic by nature
* "primitive" values only (quoted that since one can use an array)

```
$ ros2 run sample_pkg simple_pub --ros-args -p nickname:=ieda

$ ros2 param list # list params per node

$ ros 2 param get $node $param # get info about a node's param
```

```
# declare in class constructor (you can change default value to bool, int, etc
self.declare_parameter('name', 'default_value')

# rerieve it in a callback function
self.get_param('name').value

  # raises the following exception if you retrieve a missing param name
  rclpy.exceptions.ParameterNotDeclaredException
```

#### Launch Files

* Provide a way to start N nodes with different params
* Node dependency managed by packages.xml
* Launch nodes from a launch file intead of big and weird cli commands
* launch as many nodes you want (pubsub for example)
* launchers are defined in their own packages with exec deps being the packages they are supposed to launch
* always written in python
* good way to "script your deployment"
