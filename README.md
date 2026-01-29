# 🤖 RUR ROS  
## Human–Robot & Robot–Robot Interaction using Pepper Robots (ROS)

**Repository:** https://github.com/Fua6655/rur_ros  
**Platform:** SoftBank Robotics Pepper  
**Framework:** ROS (Robot Operating System)  
📺 **Demo video:** https://www.youtube.com/watch?v=QCsrS-MGEd8

---

## 🧠 Overview

This project implements a **ROS-based multi-robot system** for both  
**Human–Robot Interaction (HRI)** and **Robot–Robot Interaction (RRI)**,  
developed and deployed in a **live theatrical performance** based on  
*R.U.R. (Rossum’s Universal Robots)*.

Robot behavior is designed using a **formal behavior model based on Finite State Machines (FSM) and Behavior Tree principles**, enabling robust, predictable, and safety-aware execution in a real-world environment.

The system was executed **on physical Pepper robots**, interacting with **human actors, other robots, and the audience** in real time.

---

## 🎥 Video Demonstration

A recording from the live performance demonstrating:
- human–robot interaction  
- robot–robot coordination  
- real-time execution on stage  

👉 https://www.youtube.com/watch?v=QCsrS-MGEd8

---

## 🎭 Real-World Deployment

✔️ Live theater performance  
✔️ Human actors and audience present  
✔️ Multiple Pepper robots  
✔️ Real-time execution (no simulation)  
✔️ Safety-critical interaction  

This deployment required:
- reliable behavior transitions  
- synchronization between robots  
- predictable responses to humans  
- tolerance to timing and environmental uncertainty  

---

## 🤝 Human–Robot Interaction (HRI)

The system supports structured interaction between robots and humans:

- Interaction with human actors on stage  
- Behavior transitions triggered by human cues  
- Expressive motion, posture and timing  
- Safety-aware behavior execution in close proximity to humans  

Robots were required to:
- maintain safe distances  
- behave predictably for human performers  
- integrate interaction timing with the theatrical script  

---

## 🤖 Robot–Robot Interaction (RRI)

The project also implements **robot–robot interaction**, enabling:

- Coordination between multiple Pepper robots  
- Synchronization of behaviors and actions  
- Inter-robot communication via ROS topics  
- Shared state awareness across robots  

This transforms the system from isolated robot scripts into a **distributed multi-agent robotic system**.

---

## 🌳 Behavior Design: FSM & Behavior Tree Approach

Robot behavior is implemented using a **structured behavior model** inspired by:

- **Finite State Machines (FSM)** for clear state transitions  
- **Behavior Tree concepts** for modular, hierarchical behavior design  

This approach enables:
- deterministic and explainable robot behavior  
- safe transitions between interaction states  
- reuse of behavior modules  
- robustness in live, unstructured environments  

The FSM/Behavior Tree model proved essential for:
- handling interaction timing  
- managing concurrent human and robot interactions  
- avoiding unsafe or undefined robot states  

---

## 🧩 ROS Architecture

The system is built on a **modular ROS architecture**:

- ROS nodes for behavior execution  
- Topic-based inter-robot communication  
- Configuration-driven state transitions  
- Real-time message handling  

This architecture supports:
- scalability to additional robots  
- reuse in non-theatrical HRI scenarios  
- adaptation to other human-centered robotic applications  

---

## 📁 Repository Structure



rur_ros/
├── launch/ # ROS launch files
├── nodes/ # Interaction & behavior nodes (FSM / BT logic)
├── scripts/ # Helper scripts
├── config/ # YAML behavior and state configs
├── docs/ # Design notes and planning
└── README.md


---

## 🛠 Technical Highlights

✔️ ROS-based HRI & RRI system  
✔️ Multi-robot coordination  
✔️ FSM / Behavior Tree behavior modeling  
✔️ Real-time execution  
✔️ Safety-aware interaction logic  
✔️ Live deployment with human presence  

---

## 🧠 Why This Project Matters

This project demonstrates advanced robotics competencies:

- Human–Robot Interaction (HRI)  
- Robot–Robot Interaction (multi-agent systems)  
- Formal behavior modeling (FSM / Behavior Trees)  
- ROS ecosystem proficiency  
- Real-world deployment under uncertainty  

It shows the ability to design **robust autonomous behavior**, not just scripted motion.

---

## 🔬 Relevance for Research & EU Projects

This work directly aligns with EU priorities in:
- human-centric robotics  
- collaborative and social robots  
- safe autonomous systems  
- AI & robotics integration  

It is particularly relevant for **EU Cascade Funding calls (e.g. MAGICIAN, euROBIN)** and **short-term robotics R&D contracts**.

---

## 📌 Author

Luka Kicinbaci — Robotics / ROS Developer  
GitHub: https://github.com/Fua6655

---

## 🧠 Usage in Applications (copy-ready)

> *Developed and deployed a ROS-based multi-robot system implementing both human–robot and robot–robot interaction, using a behavior design based on FSM and Behavior Tree principles, executed in a live theatrical performance with human actors and audience.*

