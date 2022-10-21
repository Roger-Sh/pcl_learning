# PCL_Learning

个人学习 PCL (Point Cloud Library) 的仓库.



## Install Note

-   安装配置
    -   Ubuntu 18.04
    -   GPU RTX 3060 + CUDA 11.1
    -   PCL 1.9.1 + VTK 8.2.0


-   主要参考 PCL [安装教程](https://blog.csdn.net/qq_42257666/article/details/124574029)

-   安装依赖

    ```bash
    sudo apt-get update  
    sudo apt-get install git build-essential linux-libc-dev
    sudo apt-get install cmake cmake-gui
    sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
    sudo apt-get install mpi-default-dev openmpi-bin openmpi-common
    sudo apt-get install libflann1.9 libflann-dev
    sudo apt-get install libeigen3-dev 
    sudo apt-get install libboost-all-dev
    sudo apt-get install libvtk7.1-qt
    sudo apt-get install libvtk7.1
    sudo apt-get install libvtk7-qt-dev
    sudo apt-get install libqhull* libgtest-dev
    sudo apt-get install freeglut3-dev pkg-config
    sudo apt-get install libxmu-dev libxi-dev
    sudo apt-get install mono-complete
    sudo apt-get install openjdk-8-jdk openjdk-8-jre
    ```

-   安装 VTK

    -   安装依赖

    ```bash
    sudo apt-get install cmake-curses-gui
    sudo apt-get install freeglut3-dev
    ```

    -   安装 [VTK-8.2.0](https://vtk.org/download/) 
        -   mkdir build
        -   通过 cmake-gui 配置依赖
        -   选择source 和 build 路径
        -   勾选grouped advanced
        -   点击configuration 
        -   红色区域中勾选 Module_vtkGUISupportQt、VTK_Group_Qt
        -   重新 configure, generate
        -   如果有问题就删除build内容, 重新configure generate
        -   sudo make
        -   sudo make install

    

-   编译 PCL

    ```bash
    $ mkdir -p build/installed
    $ cd build
    
    $ cmake \
    > -DCMAKE_BUILD_TYPE=None \
    > -DCMAKE_INSTALL_PREFIX=./installed \
    > -DBUILD_GPU=ON \
    > -DBUILD_apps=ON \
    > -DBUILD_examples=ON \
    > ..   
    ```

    

    -   `Error generating file pcl_gpu_octree_generated_knn_search.cu.o`

        -   sm_30 sm_70 及以上不支持, 删掉 `cmake/pcl_find_cuda.cmake` 中sm_30, sm_70 及以上

            ```cmake
            if(NOT ${CUDA_VERSION_STRING} VERSION_LESS "10.0")
            	# set(__cuda_arch_bin "3.0 3.5 5.0 5.2 5.3 6.0 6.1 7.0 7.2 7.5")
            	set(__cuda_arch_bin "3.5 5.0 5.2 5.3 6.0 6.1")
            ```

            

    -   `No CMAKE_CUDA_COMPILER could be found`

        -   `.zshrc` 中添加 `export PATH=$PATH:/usr/local/cuda-11.1/bin/`





# PCL Official Tutorial

[PCL Official Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/)



## Basic Usage

### PCL Walkthrough

PCL 点云库包括以下模块:

-   Filters
    -   点云滤波, 如降采样, 过滤噪音离群点等
-   Features
    -   点云特征, 比如局部点云的法线, 弯曲度等
-   Keypoints
    -   点云关键点
-   Registration
    -   点云配准
-   KdTree
    -   Kd树
-   Octree
    -   八叉树
-   Segmentation
    -   点云分割
-   Sample Consensus
    -   样本一致性 如[RANSAC](https://zhuanlan.zhihu.com/p/402727549) 
-   Surface
    -   点云表面重建
-   Range Image
    -   深度图
-   IO
    -   点云输入输出, 如读写PCD
-   Visualization
    -   点云可视化
-   Common
    -   PCL的通用模块
-   Search
    -   点云搜索库
-   Binaries
    -   常用的PCL工具, 如pcl_viewer

### Getting Started / Basic Structures

-   PointCloud 
    -   organized point cloud 
        -   表示像图片一样分为行列的点数据
    -   projectable point cloud
        -   可以通过相机模型转换为图像UV坐标
        -   $u = fx/z, v = fy/z$
    -   `width (int)`
        -   可以指代 organized point cloud 的一行点数量, 即宽度
        -   可以指代 unorganized point cloud 的点总数
    -   `height (int)`
        -   可以指代 organized point cloud 的一列点数量, 即高度
        -   unorganized point cloud 时设置为1
    -   `points (std::vector<PointT>)`
        -   保存点云中所有的点
    -   `is_dense (bool)`
        -   表示所有点的数据都是有限的, finit (true),  Inf/NaN (fasle)
    -   `sensor_origin_ (Eigen::Vector4f)`
        -   点云传感器位姿 translation
    -   `sensor_orientation_ (Eigen::Quaternionf)`
        -   点云传感器位姿 orientation
    -   `is_Organized()`
        -   判断是否是 organized point cloud



### Using PCL in your own project

-   CMakeLists.txt 指定 PCL 安装位置

    ```cmake
    find_package(PCL 1.9.1 REQUIRED COMPONENTS common io
        PATHS "/home/shan/App/pcl/pcl-pcl-1.9.1/build/installed"
        NO_DEFAULT_PATH
    )
    ```

-   `error while loading shared libraries: libvtkCommonMisc-8.2.so.1`

    ```bash
    # 方法1: 将 /usr/local/lib 加入到 /etc/ld.so.conf, 然后执行ldconfig
    sudo chmod 777 /etc/ld.so.conf
    sudo echo "/usr/local/lib" >> /etc/ld.so.conf
    sudo ldconfig
    
    # 方法2: 临时添加到 LD_LIBRARY_PATH, 在 .zshrc 中添加以下内容
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
    
    ```

-   example: `write_pcd.cpp`

### Using a matrix to transform a point cloud

-   example: `matrix_transform.cpp`

-   转换一个点或一个向量

    ```bash
    # 点 添加1在末尾
    [10, 5, 0,  1] * 4x4_transformation_matrix
    # 向量 添加0在末尾
    [3,  0, -1, 0] * 4x4_transformation_matrix
    ```

    

## Advanced Usage

### Adding your own custom PointT type

#### PointT types available in PCL

-   PointXYZ, 

    -   member: float x, y, z

    -   通过联合体实现 SSE alignment (数据地址对齐)

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        ```

        

-   PointXYZI

    -   member: float x, y, z, intensity

    -   单独设置一个联合体用于Intensity, 因为xyz的联合体最后一项data[3]会被设置为0/1用于transformation

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        union
        {
          struct
          {
            float intensity;
          };
          float data_c[4];
        };
        ```

        

-   PointXYZRGBA

    -   member: float x, y, z; std::uint32_t rgba

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        union
        {
          union
          {
            struct
            {
              std::uint8_t b;
              std::uint8_t g;
              std::uint8_t r;
              std::uint8_t a;
            };
            float rgb;
          };
          std::uint32_t rgba;
        };
        ```

-   PointXYZRGB

    -   same as PointXYZRGBA
    -   member: float x, y, z; std::uint32_t rgba

-   PointXY

    -   member: float x, y

    -   ```
        struct
        {
          float x;
          float y;
        };
        ```

-   InterestPoint

    -   member:  float x, y, z, strength

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        union
        {
          struct
          {
            float strength;
          };
          float data_c[4];
        };
        ```

-   Normal

    -   member: float normal[3], curvature

    -   ```
        union
        {
          float data_n[4];
          float normal[3];
          struct
          {
            float normal_x;
            float normal_y;
            float normal_z;
          };
        }
        union
        {
          struct
          {
            float curvature;
          };
          float data_c[4];
        };
        ```

-   PointNormal

    -   member:  float x, y, z; float normal[3], curvature

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        union
        {
          float data_n[4];
          float normal[3];
          struct
          {
            float normal_x;
            float normal_y;
            float normal_z;
          };
        };
        union
        {
          struct
          {
            float curvature;
          };
          float data_c[4];
        };
        ```

-   PointXYZRGBNormal

    -   member: float x, y, z, normal[3], curvature; std::uint32_t rgba

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        union
        {
          float data_n[4];
          float normal[3];
          struct
          {
            float normal_x;
            float normal_y;
            float normal_z;
          };
        }
        union
        {
          struct
          {
            union
            {
              union
              {
                struct
                {
                  std::uint8_t b;
                  std::uint8_t g;
                  std::uint8_t r;
                  std::uint8_t a;
                };
                float rgb;
              };
              std::uint32_t rgba;
            };
            float curvature;
          };
          float data_c[4];
        };
        ```

-   PointXYZINormal

    -   member: float x, y, z, intensity, normal[3], curvature

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        union
        {
          float data_n[4];
          float normal[3];
          struct
          {
            float normal_x;
            float normal_y;
            float normal_z;
          };
        }
        union
        {
          struct
          {
            float intensity;
            float curvature;
          };
          float data_c[4];
        };
        ```

-   PointWithRange (depth map)

    -   member: float x, y, z (union with float point[4]), range

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        union
        {
          struct
          {
            float range;
          };
          float data_c[4];
        };
        ```

-   PointWithViewpoint

    -   member: float x, y, z, vp_x, vp_y, vp_z

    -   ```
        union
        {
          float data[4];
          struct
          {
            float x;
            float y;
            float z;
          };
        };
        union
        {
          struct
          {
            float vp_x;
            float vp_y;
            float vp_z;
          };
          float data_c[4];
        };
        ```

-   MomentInvariants

    -   member: float j1, j2, j3

    -   Simple point type holding the 3 moment invariants at a surface patch

    -   ```
        struct
        {
          float j1, j2, j3;
        };
        ```

-   PrincipalRadiiRSD

    -   member: float r_min, r_max

    -   Simple point type holding the 2 RSD radii at a surface patch

    -   ```
        struct
        {
          float r_min, r_max;
        };
        ```

-   Boundary

    -   member: std::uint8_t boundary_point

    -   Simple point type holding whether the point is lying on a surface boundary or not

    -   ```
        struct
        {
          std::uint8_t boundary_point;
        };
        ```

-   PrincipalCurvatures

    -   member: float principal_curvature[3], pc1, pc2
    -   Simple point type holding the principal curvatures of a given point.

-   PFHSignature125

    -   member: float pfh[125]
    -   Simple point type holding the PFH (Point Feature Histogram) of a given point. 

-   FPFHSignature33

    -   member: float fpfh[33]
    -   Simple point type holding the FPFH (Fast Point Feature Histogram) of a given point.

-   VFHSignature308

    -   member: float vfh[308]
    -   Simple point type holding the VFH (Viewpoint Feature Histogram) of a given point.

-   Narf36

    -   member: float x, y, z, roll, pitch, yaw; float descriptor[36]
    -   Simple point type holding the NARF (Normally Aligned Radius Feature) of a given point.

-   BorderDescription

    -   member: int x, y; BorderTraits traits
    -   Simple point type holding the border type of a given point.

-   IntensityGradient

    -   member: float gradient[3]
    -   Simple point type holding the intensity gradient of a given point.

-   Histogram

    -   member: float histogram[N]
    -   General purpose n-D histogram placeholder

-   PointWithScale

    -   member: float x, y, z, scale
    -   Similar to PointXYZI

-   PointSurfel

    -   member: float x, y, z, normal[3], rgba, radius, confidence, curvature

