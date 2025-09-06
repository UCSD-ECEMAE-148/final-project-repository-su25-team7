<div id="top"></div>

<h1 align="center"> Road Construction Detection </h1>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/UCSD-ECEMAE-148/final-project-repository-su25-team7">
    <img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/0e6cccd9-69ce-441e-9189-90d0311aa591" />
  </a>
<h3>MAE-ECE 148 Final Project</h3>
<p>
Team 7 Summer Session 2 2025
</p>
</div>

<!-- TEAM MEMBERS -->
<div align="center">
    <p align = "center"> By Kibum Kim, Owethu Kheswa,Sohailh Marie, Juncheng Li</p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#final-project">Final Project</a></li>
      <ul>
        <li><a href="#Our Approach">Our Approach</a></li>
        <li><a href="#Code Explanation">Code Explanation</a></li>
        <li><a href="#Demo Videos">Demo Videos</a></li>
      </ul>
    <li><a href="#early-quarter">Early Quarter</a></li>
      <ul>
        <li><a href="#mechanical-design">Mechanical Design</a></li>
        <li><a href="#electronic-hardware">Electronic Hardware</a></li>
        <li><a href="autonomous-laps: donkeycar laps">Autonomous Laps donkeycar</a></li>
        <li><a href="autonomous-laps: gps laps ">Autonomous Laps gps</a></li>
        <li><a href="autonomous-laps: ros2 laps">Autonomous Laps ros2</a></li>
      </ul>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

<h4>Team Member Major and Class </h4>
<ul>
  <li>Kibum Kim - Electrical Engineering (EC27) - Class of 2026</li>
  <li>Owethu Kheswa - Mechanical & Aerospace Engineering (MC34) - Class of 2026</li>
  <li>Sohailh Marie -  Mechanical & Aerospace Engineering (MC34) - Class of 2026</li>
  <li>Juncheng Li - Mechanical & Aerospace Engineering (MC34) - Class of 2026</li>
</ul>

<!-- Final Project -->
## Final Project

<div align="center">
    <img src="images\team7_robot.png" alt="Logo" width="500" height=400">
    <img src="images\team7_track.jpg" alt="Logo" width="500" height=400">
</div>

The goal of the project was to design a robot that can do the following on a track with two yellow dotted lanes:
<ul>
  <li>Follow the path of yellow lines.</li>
  <li>Detect construction ahead and decide how to avoid it. If the construction is in one lane, switch to the other lane. If the road is blocked off (dead end), make a U-turn and go back.</li>
  <li>Detect pedestrians with shoes on the track and stop a distance before them.</li>
</ul>

### Our Approach:
1) LiDAR detects objects in front of the car and determines if the obstacle is blocking a lane or both lanes. Then it move around the obstacle or makes a U-turn.
2) Recycle the Lane Detection Feature from the class assignment (please refer to the ros2 laps section) to make the car move inside the track if no obstacles are ahead.
3) Incorporate an open-source AI model that detects shoes and use the model to detect pedestrians in front and stop.

### Code Explanation:
<div align="center">
    <img src="images\project_node_flow.png" alt="Logo" width="700" height=400">
</div>

The robot functions using three main nodes: the LiDAR Detection node, the Lane Detection node, and the Priority Node.
<ul>
  <li>LiDAR Detection node = subscribed to the LiDAR data publishing node. Every time an object is detected 2 meters away, stop the robot and analyze where the obstacle(s) are. Make the VESC move according to the analysis. Publishes vesc Twist messages to Priority Node.</li>
  <li>Lane Detection node = subscribed to the OAKD camera data publishing node. Uses the camera captures to find the dotted lanes using an AI detection model and adjust s the vesc control accordingly. Publishes vesc Twist messages to Priority Node. </li>
  <li>Priority node = Subscribed to both the LiDAR Detection and Lane Detection node. Determines which VESC Twist messages to prioritize and transfer to the VESC. Prioritizes the Twist messages from the LiDAR Detection node over the Lane Detection node. Publishes vesc Twist messages to the VESC node.</li>
</ul>

### Demo Videos & Other Resources
<div align="center">
    [<img src="images\robot_switching_lanes.mov" width="50%">](robot switching lanes)
</div>
<div align="center">
    [<img src="images\robot_making_uturn.mov" width="50"%>](robot making U-turn)
</div>

* [Final Project Proposal](https://docs.google.com/presentation/d/1GVsvTshFEmerzxDG04KnkVLLrKt4Ls0ByvFJtbYKdKk/edit?slide=id.p#slide=id.p)
* [Final Project Presentation](https://docs.google.com/presentation/d/1y8KrdfWulMwEmKlMkTtelip-Z3ugiJRzll6btDa6zmI/edit?slide=id.g2930274da63_0_126#slide=id.g2930274da63_0_126)


## Early Quarter

### Mechanical Design



