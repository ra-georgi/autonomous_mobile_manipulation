## Autonomous Mobile Manipulation

![Navigation and manipulation](media/demo_navigation.gif)

**Gazebo (GZ Sim) simulation of an autonomous mobile manipulator** combining mobile navigation, manipulation, and task-level autonomy.

A **Franka Panda** robotic arm is mounted on a custom **differential-drive mobile base**. The system integrates:

- **Nav2** for autonomous navigation  
- **MoveIt 2** for motion planning and manipulation  
- A **behavior tree** for task-level decision making and action sequencing  

The robot reacts to objects spawned in the environment, autonomously navigates to them, grasps them, and places them into the appropriate container mounted on the mobile base.

This project focuses on **end-to-end autonomy**, showcasing the interaction between perception (placeholder), navigation, manipulation, and high-level control in simulation.

![Autonomous pick and place](media/demo_pick_place.gif)

---

## Roadmap / Future Work

- Tune **Nav2** and **MoveIt 2** parameters for faster and smoother operation  
- Add a **second autonomous agent** to the simulation  
- Add a **camera sensor** to the robot model and replace the placeholder perception node with vision-based feedback  

