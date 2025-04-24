# [S^3-Slicer: A General Slicing Framework for Multi-Axis 3D Printing](https://guoxinfang.github.io/S3_Slicer) (Technical Papers' [Best Paper Award](https://sa2022.siggraph.org/en/attend/award-winners/index.html))

![](DataSet/figures/teaser.jpg)

[Tianyu Zhang](https://www.linkedin.com/in/tianyu-zhang-49b8231b5/), Guoxin Fang, Yuming Huang, Neelotpal Dutta, Sylvain Lefebvre, Zekai Murat Kilic, and [Charlie C.L. Wang](https://mewangcl.github.io/), [*ACM Transactions on Graphics (SIGGRAPH Asia 2022)*, vol.41, no.6, article no.277 (15 pages), December 2022](https://dl.acm.org/doi/10.1145/3550454.3555516)

## Abstract
Multi-axis motion introduces more degrees of freedom into the process of 3D printing to enable different objectives of fabrication by accumulating materials layers upon curved layers. An existing challenge is how to effectively generate the curved layers satisfying multiple objectives simultaneously. This paper presents a general slicing framework for achieving multiple fabrication objectives including support-free, strength reinforcement and surface quality. These objectives are formulated as local printing directions varied in the volume of a solid, which are achieved by computing the rotation-driven deformation for the input model. The height field of a deformed model is mapped into a scalar field on its original shape, the isosurfaces of which give the curved layers of multi-axis 3D printing. The deformation can be effectively optimized with the help of quaternion fields to achieve the fabrication objectives. The effectiveness of our method has been verified on a variety of models. [Video Link](https://www.youtube.com/watch?v=qNm1ierKuUk)

The extension work "S4 Slicer: Simple S3 Slicer" develped by @Joshua Bird is also available.[[github]](https://github.com/jyjblrd/S4_Slicer) [[Video@YouTube]](https://www.youtube.com/watch?v=M51bMMVWbC8)

## Installation

Please compile the code with QMake file “ShapeLab.pro”.

**Platform**: Windows + Visual Studio + QT-plugin (tested version: VS2019 + QT5.12.3 + msvc2017_64)

**Install Steps**: 
- **Install Visual Studio Extension plug-in (QT VS Tool)** to open the *.pro file and generate the project
- **Set 'ShapeLab' as the start up project**
- **Enable OpenMP to get best performace** at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Language -> Open MP Support -> Select '**Yes (/openmp)**'
- **Open Console** at: ShapeLab Property Pages -> Configuration Properties -> Linker -> System -> SubSystem -> Select '**Console (/SUBSYSTEM:CONSOLE)**'
- **Support large obj** at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Command Line -> Additional Options -> add "/bigobj"
- **Install Intel oneAPI Math Kernel Library (oneMKL [download](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html?operatingsystem=window&distributions=online))** and enable it at: ShapeLab Property Pages -> Configuration Properties -> Intel Libraries for oneAPI -> Intel oneAPI Math Kernel Library (oneMKL) -> Use oneMKL -> Select '**Parallel**'
- **And** change the code generation method at: ShapeLab & QMeshLab & GLKLib Property Pages -> Configuration Properties -> C/C++ -> Code Generation -> Runtime Library -> Select '**Multi-threaded(/MT)** for release configuration'. Note that this option will be '**Multi-threaded Debug (/MTd)** for debug configuration.
- Please ensure the MKL is added into the **Environment Variables** (e.g. C:\Program Files (x86)\Intel\oneAPI\compiler\2024.1\bin)

![](DataSet/figures/pipline.jpg)

## Usage
**Step 0: Input tetrahedron mesh into the system**
Click button **Open** at the left-up corner of UI. For example, **bunny_cut6** is opened.
- **Note**: If you want to try out a new model, please make sure to generate the required *.tet model files and stress field files. See the [documentation](Pre_processing.pdf) for details.

**Step 1: Run fabrication objective-driven deformation**
Click button **1.x fabrication objective (Support free - SF, Strength reinforcement - SR, and Surface quality - SQ)** at the right side of UI. For example, **1.7 SF_SR_SQ** is enabled for **bunny_cut6** model.

**Step 2: Run inverse deformation**
Click button **2. Inverse Deformation** to resume the deformed model and generate the scalar field for slicing.

**Step 3: Curved layer slicing**
Move to the next page **Slicing_Toolpath**, click button **3.0 Layer Generation (scalar)** to generate curved layers, and use **Output** to save the layers into **\DataSet\remesh_operation\layers_unremeshed**.

- **Note**: The button **3.1 Adaptive Height Slicing** is used for **AnkleBaseV1** model, the rest given model will use button 3.0.

**Step 4: Curved layer remesh**
The remesh operation is conducted by running .bat file integrated in the MeshLab.

- **Note**: The **remesh_slimmedLayer.bat** file is in **\DataSet\remesh_operation**, and the directory line should be modified. Please check this github link for more details [here](https://github.com/zhangty019/remesh_bat).

**Step 5: Toolpath generation**
Click button '**Get Tool-Path**' to generate toolpath with parameters - Width and Distance. And **Output** to save the layers and toolpath into **\DataSet\CURVED_LAYER** and **\DataSet\TOOL_PATH**.

**Step 6: Singularity-Aware Motion Planning for Multi-Axis Additive Manufacturing**
Please refer to this project [link](https://github.com/zhangty019/MultiAxis_3DP_MotionPlanning) for the G-code generation.

![](DataSet/figures/printingResult.jpg)

**Note**: Please refer to the following articles for information on generating the necessary curved supports and toolpaths along the stress direction.
1) the support generation is based on 
Tianyu Zhang, Yuming Huang, Piotr Kukulski, Neelotpal Dutta, Guoxin Fang, and Charlie C.L. Wang, "[Support generation for robot-assisted 3D printing with curved layers](https://ieeexplore.ieee.org/abstract/document/10161432)", IEEE International Conference on Robotics and Automation (ICRA 2023), London, United Kingdom, May 29 - June 2, 2023.

2) the toolpath generation on surface is based on
Guoxin Fang, Tianyu Zhang, Sikai Zhong, Xiangjia Chen, Zichun Zhong, and Charlie C.L. Wang, "[Reinforced FDM: Multi-axis filament alignment with controlled anisotropic strength](https://dl.acm.org/doi/abs/10.1145/3414685.3417834)", ACM Transactions on Graphics (SIGGRAPH Asia 2020), vol.39, no.6, article no.204 (15 pages), November 2020.

## Contact Information
Tianyu Zhang      (tianyu.zhang@manchester.ac.uk)

Tao Liu           (tao.liu-2@postgrad.manchester.ac.uk)

Charlie C.L. Wang (changling.wang@manchester.ac.uk)
