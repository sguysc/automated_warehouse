#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/nmea_navsat_driver"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/install/lib/python2.7/dist-packages:/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build" \
    "/usr/bin/python" \
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/nmea_navsat_driver/setup.py" \
    build --build-base "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/nmea_navsat_driver" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/install" --install-scripts="/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/install/bin"
