# ROS2 port of monocular object detection package

This is a ROS 2 port of Benchun Zhou`s ROS1 [monocular_object_detection](https://github.com/benchun123/monocular_object_detection) package from his [point-plane-object-SLAM]() paper. My goal was to keep this implementation as close to the original as possible. Hence, it mostly reuses the varialbes and function calls but differs from how Thirdparty dependencies are handled. If you use this package for academic work, please consider citing the original author's paper below

** Benchun Zhou, Maximilian Gilles, Yongqi Meng. **Structure SLAM with Points, Planes, and Objects**[J]//Advanced Robotics 36.20 (2022): 1060-1075. [[**Link**](https://www.tandfonline.com/doi/full/10.1080/01691864.2022.2123253)] [[**PDF**](./README_Picture/2022_Advanced_Robotics_Publication.pdf)]  [[**Slide**](./README_Picture/2022_Advanced_Robotics_Slide.pdf)]
 [[**Youtube**](https://youtu.be/nBbGTFeUh88)] [[**Bilibili**](https://www.bilibili.com/video/BV1JM4y167uT)]


## 0. Some notable changes
* OpenCV >=4.2, Boost >=1.80, CXX17 standard, Eigen >=3.3, PCL 1.13.1 (exactly, see below), Cmake>=3.5, and two other ros2 packages
* Change some OpenCV keywords to reflect the changes made between OpenCV 3 and OpenCV 4
* The original ROS 1 package used PCL 1.8.1 but Ubuntu 22.04 is compatible with 1.12.1. In version 1.12, ```boost::make_shared<pcl::PointIndices>``` is changed to ```pcl::make_shared<pcl::PointIndices>```.  
* Solved this [error](https://github.com/PointCloudLibrary/pcl/issues/5063) that occurs due to PCL 1.21.1 library being shipped out with Ubuntu 22.04 before [PR #5130](https://github.com/PointCloudLibrary/pcl/pull/5130) was commited to PCL library
* Tested with newest PCL release (1.14) but starting from 1.14 but was having issue with the ```sample consensus``` library in PCL. Hence, downgraded to 1.13.1.
* If I am not mistaken, starting from 1.13, PCL library requries Boost>=1.82 due to a change in how ```filesystems``` library is used. Will add a better info source if I come accross it. 
* For newcomers in ROS2 ecosystem, this package serves as an example of building a shared cpp library and on using so-called **library-only** (my own opinion, don`t quote me on this) ROS2 packages.

## Tested with
* Ubuntu 22.04 LTS (Jammy Jellyfish)
* ROS2 Humble Hawksbill (LST)
* Boost 1.84
* PCL 1.13.1
* Cmake 3.8

## 1. Prepare dataset
Install ```gdown``` if you don't have it
```
pip3 install gdown
```
Then download the sample SUN RGB-D dataset provided in the original ROS1 repository in /Documents folder. 
```
cd ~/Documents
gdown --id 14PQWSmCsBvmomllWeF4_hlDiEuYIyWgQ # Download from Google drive
unzip SUN_RGBD_Selected.zip 
```
> [!WARNING]  
> The dataset must exsist here, I hardcoded its location.

## 2. Install prerequisiti softwares

### Eigen 3
```
sudo apt install libeigen3-dev
```

### Boost 1.84
Uninstall old version of boost. Adopted from this [stackexchange](https://stackoverflow.com/questions/8430332/uninstall-boost-and-install-another-version) post.
> [!CAUTION]
> Go over the linked stackexchange post before doing this step.
```
sudo apt-get -y --purge remove libboost-all-dev libboost-doc libboost-dev
echo "clear boost dir"
sudo rm -r /usr/local/lib/libboost*
sudo rm -r /usr/local/include/boost
sudo rm -r /usr/local/lib/cmake/[Bb]oost*
sudo rm -f /usr/lib/libboost_*
sudo rm -r /usr/include/boost
```
Now lets download and install Boost 1.84. Steps adopted from the same stackexchange post liked above
```
cd ~\Documents # Move to Documents folder
mkdir boost && cd boost
wget http://downloads.sourceforge.net/project/boost/boost/1.84.0/boost_1_84_0.tar.gz # Download boost 1.84
tar -zxvf boost_1_84_0.tar.gz
cd boost_1_84_0
# get the no of cpucores to make faster
cpuCores=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'`
echo "Available CPU cores: "$cpuCores
./bootstrap.sh  # this will generate ./b2
sudo ./b2 --with=all -j $cpuCores install
```

Then check if its installed correctly
```
cat /usr/local/include/boost/version.hpp | grep "BOOST_LIB_VERSION"
```
which should give an output similar to 
```
//  BOOST_LIB_VERSION must be defined to be the same as BOOST_VERSION
#define BOOST_LIB_VERSION "1_84"
```

### Point Cloud Library 1.13.1

Remove old point cloud library

```
sudo apt remove libpcl-dev

```
> [!WARNING]
> There is high possibility this step will result in removal of a good chunk of ROS 2 Humble packages. In that case, reinstall ROS 2 Humble once you are done with installing prerequisit software

Now download and install PCL 1.13.1. Steps adopted from [official tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html)
```
cd ~/Documents
mkdir pcl_library && cd pcl_library
wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.13.1.tar.gz
cd pcl-pcl-1.13.1 && mkdir build && cd build
cmake .. # or ```ccmake ..``` if you want to change some settings.
```
> [!IMPORTANT]  
> At this stage, if you try to install, some of the PCL libraries will fail to build since Cmake created does not look for Boost 1.84. To solve this, do the following
> In the ```pcl-pcl-1.13.1``` directory find ```PCLConfig.cmake.in``` file and change ```set(Boost_ADDITIONAL_VERSIONS)``` to the following
```
set(Boost_ADDITIONAL_VERSIONS
    "@Boost_MAJOR_VERSION@.@Boost_MINOR_VERSION@.@Boost_SUBMINOR_VERSION@" "@Boost_MAJOR_VERSION@.@Boost_MINOR_VERSION@"
    "1.84.0" "1.84" "1.83.0" "1.83" "1.82.0" "1.82" "1.81.0" "1.81" "1.80.0" "1.80"
    "1.79.0" "1.79" "1.78.0" "1.78" "1.77.0" "1.77" "1.76.0" "1.76" "1.75.0" "1.75" 
    "1.74.0" "1.74" "1.73.0" "1.73" "1.72.0" "1.72" "1.71.0" "1.71" "1.70.0" "1.70"
    "1.69.0" "1.69" "1.68.0" "1.68" "1.67.0" "1.67" "1.66.0" "1.66" "1.65.1" "1.65.0" "1.65")
```

Now finish installing PCL library as follows
```
sudo make -j4 # If you have more cores, you can choose a larger number
sudo make -j4 install
```

## 3a. Prepare a ros2 workspace [Optional but I recommend it]
```
cd ~
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_test/src
```

## 3b. Source and build two ros2 packages
```
cd ~/ros2_test/src
git clone https://github.com/Mechazo11/ros2_tictoc_profiler.git
git clone https://github.com/Mechazo11/ros2_line_lbd.git
cd .. # move to root directory of ros2_test
colcon build --packages-select ros2_tictoc_profiler ros2_line_lbd
source install/setup.bash # When installation is successful
```

## 3c. Source and build this package
```
cd ~/ros2_test/src
git clone https://github.com/Mechazo11/ros2_monocular_object_detection.git
cd ..
colcon build --packages-select ros2_monocular_object_detection
source install/setup.bash
```

## 4. Testing
Run the following command and press any keys to move through the sample proposals and final cuboid proposal for each image.
```
ros2 run ros2_monocular_object_detection det_rgbd_opti_sun_node 
```

## 5 Known bugs
** Cannot shutdown the node with Ctrl+C.
