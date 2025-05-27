# Strawberry-field

# **About this work**  

This work is an example of *software-in-the-loop* (SIL), where trajectory planning is performed within a strawberry field using a visual memory. The process starts with a camera that compares a reference image with a desired image (the latter are stored in the visual memory). 

First, ORB points are detected in both images, and then a point correspondence is established to gather information and compute the homography matrix. This matrix provides data on rotation and translation. Finally, this information is used to integrate it into the IBVS control, as presented in [1].

<p align="center">
  <img src="world.jpg" alt="mundo" />
</p>

---

# **How to Run the Program**  

## **Prerequisites**  

You need to install OpenCV first. You can install it with the following command:

```bash
sudo apt-get update
sudo apt-get install libopencv-dev
```

You need to install the TurtleBot3 packages in order to use the TurtleBot in Gazebo:

- [TurtleBot3 Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [TurtleBot3 Simulation Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

You also need to install the ViSP library:

- [ViSP Installation Guide for Ubuntu](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html)

Additionally, install the ViSP ROS package:

- [ViSP ROS GitHub Repository](https://github.com/lagadic/visp_ros)

### Setup Instructions  

1. **Download the source code** and place it inside a ROS workspace.  
2. **Compile the workspace** by running:  

    ```bash
    catkin_make
    ```

3. **Modify the image reference path:**  
   - Open `homography_vision.cpp`.  
   - Change the path on **line 154** to the full path where the `image_reference` folder is located.  

4. **Run the program** with the following commands:  

    ```bash
    source devel/setup.bash
    export TURTLEBOT3_MODEL=turtlebot3_waffle1
    roslaunch turtlebot3_gazebo turtlebot3_fresa.launch
    ```  

### (Optional) Visualization  

If you want to visualize the point correspondences and the desired image transformation:  

- Open a new terminal in the same workspace directory.  
- Run the following command:  

    ```bash
    rqt_image_view
    ```  

- Look for the topic `/camera/rgb/vision_image_matches` in the list.  


**Note:** The models used for the strawberry field environment were added to  
`turtlebot3_simulations/turtlebot3_gazebo/models`.

<p align="center">
  <img src="cultivo.gif" alt="Strawberry" />
</p>

This program was developed and tested in **ROS 1 Noetic**. 🚀

---

## Contributors  

- [@erandivg](https://github.com/erandivg)  
- [@gfloresc](https://github.com/gfloresc)  
- [@NoePity2](https://github.com/NoePity2)

<p align="center">
  <a href="https://github.com/erandivg/strawberry_field_ws/graphs/contributors">
    <img src="https://contrib.rocks/image?repo=erandivg/strawberry_field_ws" />
  </a>
</p>

---

## References  

[1] Chaumette, F., & Hutchinson, S. (2006). **Visual Servo Control, Part I: Basic Approaches**. *IEEE Robotics & Automation Magazine*, 13(4), 82–90. [Link](https://inria.hal.science/inria-00350283v1/document)  

[2] **Implementación De Control Visual Para Planificación De Trayectorias En Un Cultivo De Fresas Virtual**. *Jóvenes en la Ciencia*. Universidad de Guanajuato. [Link](https://www.jovenesenlaciencia.ugto.mx/index.php/jovenesenlaciencia/article/view/4691)  

[3] **Visual Control Based Trajectory Design for a Mobile Robot in an Agricultural Environment**. Master’s Thesis. Centro de Investigaciones en Óptica, A.C. (CIO), 2025. [Link](https://cio.repositorioinstitucional.mx/jspui/handle/1002/1357)  

