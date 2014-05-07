thorp_bringup
=========

#### Pre-requisite ####

```
sudo pip install -U yujin_tools
```

#### Installation ####

```
yujin_init_workspace -j10 --track=hydro ~/thorp_bringup https://raw.github.com/yujinrobot/kobuki-x/hydro-devel/thorp_bringup/thorp_bringup.rosinstall
cd ~/thorp_bringup
source /opt/ros/hydro/setup.bash
rosdep update
rosdep install --from-paths src /opt/ros/hydro --ignore-src --rosdistro hydro -y
yujin_init_build . -u /opt/ros/hydro
yujin_make
```

#### Prepare hardware ####

IR sensors ring requires a bit of extra work. Check their [launch file](https://github.com/yujinrobot/kobuki-x/blob/hydro-devel/thorp_bringup/launch/includes/_ir_sensors.launch) for instructions.
