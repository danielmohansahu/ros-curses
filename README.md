# ros-curses

`ros_curses` is still in active development, and subject to large changes! Use at your own risk.

`ros-curses` is a command line interface for debugging live [ROS](https://www.ros.org/) applications. It is based on [ncurses](https://en.wikipedia.org/wiki/Ncurses). This project was prompted by the frustration of the author in repeatedly running successive commands like `rostopic info ...`, `rosnode info -q ...`, etc.

### Installation

Despite the fact that this tool interfaces with ROS, the number of dependencies are quite light. In addition to `ncurses` we also use [xmlrpc](http://xmlrpc.com/) for ROS querying. To install all required dependencies run the following (or equivalent on your OS):

```bash
apt install cmake libxmlrpcpp-dev libncurses-dev
```

That's it! To build, run the following:

```bash
# build the package
mkdir build -p && cd build
cmake .. && make

# install, if desired
sudo make install
```

### Execution

Run the executable via `ros-curses` (or `./ros-curses` if not installed). The GUI should appear, and, if you have a live ROS instance running show something like the following:

![roscore_example](./docs/roscore_example.gif)

### Notes

Currently only ROS1 is supported. I've architected things to make adding a ROS2 parser _somewhat_ easy, but some fundamental changes will be needed for representing the computational graph.

Feature Roadmap:
 - Flesh out "Info" panel displays to show more information (e.g. params in Node namespace)
 - Add struct definitions where applicable (Topics, Services)
 - Intuitive highlighting of common issues (unsubscribed topics, etc.)
 - Add basic Topic panel tools, like an equivalent of `rostopic (hz, bw)`. Not `echo`.
 - optionally display 'tree' view (i.e. separate namespaces)




