#!/bin/bash
# Runs once per `docker run`/`docker compose up`, after thorp is bind-mounted, to redo the
# bits that the deployment Dockerfile does at image-build time (when thorp is actually
# present on disk). Safe to run every time: everything here is idempotent.
#
# Nothing in here is allowed to stop the container from starting: a flaky apt mirror or no
# network at all should cost you a warning, not a shell you can't get into. Hence no `set -e`
# and the explicit guards below.

# Configure the workspace exactly as the deployment build configures it. The Dockerfile does
# this too, but doing it here as well is what makes it reliable: the profile lives in
# /catkin_ws/.catkin_tools, which is part of the container rather than one of the mounted
# volumes, so it's gone as soon as a container is (say, anything run with --rm). Doing it on
# every start also means a container from an older image still gets the right flags.
# These aren't optional: thorp_toolkit's common.hpp uses std::optional, so without
# -DCMAKE_CXX_STANDARD=17 the workspace doesn't compile at all (gcc 9 defaults to gnu++14).
if [ -d /catkin_ws/src ]; then
  source "/opt/ros/${ROS_DISTRO:-noetic}/setup.bash"
  if (cd /catkin_ws && catkin config --init --extend "/opt/ros/${ROS_DISTRO:-noetic}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 > /dev/null); then
    echo "catkin workspace configured: Release, C++17"
  else
    echo "warning: catkin config failed -- build with" \
         "--cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 by hand" >&2
  fi
fi

if [ -d /catkin_ws/src/thorp ]; then
  # thorp's own package.xml files aren't visible at image build time (thorp isn't cloned
  # into the dev image, it's bind-mounted here), so rosdep only resolved the OTHER cloned
  # repos' dependencies back then. Do a second, fast pass now that thorp is mounted, so
  # whatever its manifests need (nlohmann-json-dev, behaviortree_cpp, cob_perception_msgs,
  # or anything added later) gets installed without hand-maintaining that list in the
  # Dockerfile. Cheap when nothing changed: rosdep/apt skip already-installed packages.
  export DEBIAN_FRONTEND=noninteractive
  if apt-get update -qq; then
    rosdep install -y -r --rosdistro="${ROS_DISTRO:-noetic}" --from-paths /catkin_ws/src --ignore-src \
        --skip-keys='create_node create_dashboard create_description \
                     rocon_app_manager kobuki_rapps kobuki_capabilities turtlebot_capabilities \
                     stdr_gui stdr_robot stdr_server astra_launch realsense_camera zeroconf_avahi' \
        || echo "warning: rosdep install had issues -- see output above" >&2
  else
    echo "warning: apt-get update failed, skipping the rosdep pass for thorp's own deps" >&2
  fi

  # remake symbolic links for the small house world, if both source and destination exist
  if [ -d /catkin_ws/src/small_house_world ] && [ -d /catkin_ws/src/thorp/thorp_navigation ]; then
    ln -sfnT /catkin_ws/src/small_house_world/maps/turtlebot3_waffle_pi/small_house.png \
        /catkin_ws/src/thorp/thorp_navigation/maps/small_house.png
    ln -sfnT /catkin_ws/src/small_house_world/maps/turtlebot3_waffle_pi/small_house.yaml \
        /catkin_ws/src/thorp/thorp_navigation/maps/small_house.yaml
    ln -sfnT /catkin_ws/src/small_house_world/worlds/small_house.world \
        /catkin_ws/src/thorp/thorp_simulation/worlds/gazebo/small_house.world
  fi
else
  echo "=============================================================================" >&2
  echo "WARNING: nothing mounted at /catkin_ws/src/thorp -- there is no thorp source"  >&2
  echo "in this container, so catkin has nothing of yours to build."                   >&2
  echo                                                                                 >&2
  echo "The bind mount lives in docker-compose.dev.yml, not in the image, so a plain"   >&2
  echo "\`docker run\` won't set it up. Either:"                                        >&2
  echo "  docker compose -f docker-compose.dev.yml run --rm thorp-dev"                  >&2
  echo "or pass it yourself:"                                                           >&2
  echo "  docker run ... -v /path/to/thorp:/catkin_ws/src/thorp thorp:noetic-dev"       >&2
  echo "=============================================================================" >&2
fi

exec "$@"
