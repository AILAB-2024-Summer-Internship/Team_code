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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/usr/local/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/usr/local/lib/python3/dist-packages:/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/build" \
    "/home/ailab/anaconda3/bin/python3" \
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/setup.py" \
     \
    build --build-base "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/build" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/usr/local" --install-scripts="/usr/local/bin"
