### Staubli ROS2 control stack

## Meta-package `staubli_driver_ros2`

This is a meta-package for the Staubli ROS2 control stack.

## How to build the documentation locally?

1) Make sure sphinx and doxygen are installed

```bash
source /opt/ros/<distro>/setup.bash
sudo apt install doxygen doxygen-gui doxygen-doc
```

2) Install the necessary sphinx extensions in a virtual environment

```bash
cd staubli_driver_ros2/staubli_driver_ros2

python3 -m venv .venv
touch .venv/COLCON_IGNORE
source .venv/bin/activate

pip install -r requirements.txt
```

3) Build the documentation

```bash
sphinx-build -b html sphinx sphinx/_build
# Or to force a full re-build of the docs
sphinx-build -E -a -b html sphinx sphinx/_build
```

4) Open local documentation

```bash
firefox sphinx/_build/index.html
```

5) (Opt.) Build PDF documentation

```bash
sphinx-build -E -a -b latex sphinx pdf/latex
pdflatex \
    -output-directory=pdf/latex \
    pdf/latex/staubli_driver_ros2.tex
firefox pdf/latex/staubli_driver_ros2.pdf
```
