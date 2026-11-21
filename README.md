# Explainable Kitchen Robot — Domestic Object Transportation

Course project for **Explainable Cyber-Physical Systems** (Carl von Ossietzky Universität Oldenburg,
WS 2026/27).

## What this project is about

A mobile robot sets the table in a kitchen: it finds milk, cereal and a bowl on the counter, takes them,
carries them to the kitchen island and sets them down there — or clears the table again. Robots like
this make many decisions on their own (where to stand, where to look, what to do after a failure),
and people watching them often cannot tell what the robot is doing or why.

The goal of the project is to make the robot **explainable**: to find out which explanations are needed,
for whom and in which situations, and to let the robot give them — so that people understand it and
trust it as much as it deserves. The work follows four steps:

1. **Exemplar and scenario** — the *Domestic Object Transportation* lab of the AICOR Virtual Research
   Building (University of Bremen): a PR2 robot in the IAI kitchen, action core *Transporting*,
   specialisations *setting the table* and *cleaning the table*.
2. **Explainability requirements** — derived with a requirements-engineering process: stakeholders
   (user, people sharing the kitchen, developer), guiding questions, requirements and design decisions.
3. **Situations that need explanations** — classified with the taxonomy of explanation needs
   (context-aware behaviour, failure, disobedience) and filtered by complexity, cost and occurrence.
4. **Implementation and evaluation** — an explanation layer on top of the robot's task plan, evaluated
   with a small user study.

This repository contains the **simulation the explanations are built on**: a Gazebo / ROS 2 version
of the exemplar. The original lab runs in remote Jupyter notebooks with a kinematic simulation, where
the robot jumps between poses and can move through furniture. Here the same kitchen, objects and task
run with real physics and real navigation, and every robot action reports what happened and why.

## What the simulation does

- **Kitchen and robot:** the lab's IAI kitchen model and the lab's PR2 model in Gazebo Harmonic.
- **Navigation:** Nav2 drives the robot between the sink counter and the kitchen island on a map
  generated from the kitchen model.
- **Perception:** a simulated head camera detects an object when it is inside the camera's field of view.
- **Carrying:** the robot does not grasp with its arms. When it has seen an object within reach, it
  loads it onto a carry tray in front of its torso, drives to the goal and sets it down upright —
  after checking that the goal is within reach and free. Taking and setting down are pose changes in
  the simulator, not physics; on the tray a fixed joint holds the object while the robot drives. The
  result is checked against the object's goal (5 cm tolerance). Grasping with the arms is not needed
  for the project (decided with the professor), so the arms stay parked.
- **Task logic:** a behaviour tree (BehaviorTree.CPP) for *set the table* and *clean the table*.
  Each action calls one robot skill.
- **Explanations (first version):** every skill answers with what happened and why, for example
  *"breakfast_cereal is 12 deg right of the camera's field of view (the head points elsewhere)"*. After each run the
  runner writes a summary: *What did the robot do? Did anything go wrong, and why? Result.*

Results of a full run from a fresh start (simulated time, two runs in a row):

| Scenario | Result | Time | Distance of milk / cereal / bowl from their goals |
|----------|--------|------|---------------------------------------------------|
| set the table (counter → island) | success | 106 s (both runs) | 0.0 / 0.1 / 0.2 cm |
| clean the table (island → counter) | success | 83–88 s | 0.0 / 0.1 / 0.2 cm |

Typical explanations when something is not possible (each is the skill's own answer):

- *"the milk has not been seen yet; the robot must detect it before taking it"*
- *"the tray already carries the milk; it holds one object at a time"*
- *"the goal spot on the counter is occupied: the bowl stands 0 cm from it, the milk would touch it"*
- *"the goal (-1.40, 1.20) on the island is 1.27 m from the robot centre, beyond its 0.85 m reach (0.42 m too
  far; the robot must stand closer)"*

## How to run it

### Requirements

- Ubuntu 24.04 (native or WSL 2 on Windows) with a graphics driver that supports OpenGL
- ROS 2 Jazzy and Gazebo Harmonic
- About 8 GB RAM; a full run takes 3–5 minutes

Install the dependencies once:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                 ros-jazzy-behaviortree-cpp python3-colcon-common-extensions python3-numpy python3-yaml
```

### Quick start

```bash
git clone https://github.com/AmirHosseinA47/XCPS---Domestic-Object-Transportation-Robot.git
cd XCPS---Domestic-Object-Transportation-Robot
bash run_demo.sh set      # build, start the simulation, set the table
```

`run_demo.sh` builds the workspace, starts Gazebo, Nav2, RViz and the robot skills, and then runs the
behaviour tree. Press Enter at the end to stop the simulation. To clear the table instead, run
`bash run_demo.sh clean` (the objects must be on the island, e.g. after a `set` run in the same
simulation — see the step-by-step commands below).

### Step by step

Build once, then use a new terminal for each command (each terminal needs the two `source` lines):

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

```bash
# Terminal 1: simulation + Nav2 + robot skills (wait for "kitchen skills ready")
ros2 launch kitchen_gazebo kitchen_tasks.launch.py

# Terminal 2: the behaviour tree
ros2 run kitchen_bt kitchen_bt_runner --scenario set      # then: --scenario clean
#   --groot   also stream the tree live to Groot2 (port 1667)
```

Other entry points:

```bash
ros2 launch kitchen_gazebo kitchen_sim.launch.py              # only the robot in the kitchen
ros2 launch kitchen_gazebo kitchen_sim.launch.py gui:=false   # same, without the Gazebo window
ros2 launch kitchen_gazebo kitchen_nav.launch.py              # simulation + Nav2 ("2D Goal Pose" in RViz)
ros2 run kitchen_gazebo nav_test.py --rounds 2                # drive counter <-> island and log it
ros2 run kitchen_gazebo skills_demo.py --scenario set         # the same task without the behaviour tree
```

Call a single robot skill (skills: `state`, `park_arms`, `move_torso`, `navigate`, `look_at`,
`detect`, `pick`, `place`, `object_pose`):

```bash
ros2 service call /kitchen_skills/execute kitchen_interfaces/srv/Skill \
  "{skill: navigate, params_json: '{\"to\": \"counter\"}'}"
```

### Output

| Where | What |
|-------|------|
| `~/kitchen_bt_runs/<scenario>_<time>.jsonl` | every behaviour-tree status change and skill answer |
| `~/kitchen_bt_runs/<scenario>_<time>_explanation.txt` | the run summary: what the robot did, what went wrong and why, result |
| `~/kitchen_bt_runs/<scenario>_<time>.btlog` | behaviour-tree log that Groot2 can replay |
| `/kitchen_skills/events` (ROS topic) | every skill answer, live |
| `~/kitchen_tasks.log` | simulation log when started with `run_demo.sh` |

### Troubleshooting

- **The simulation is very slow or Gazebo shows no window (WSL):** update WSL (`wsl --update`) and the
  Windows graphics driver; `gui:=false` runs Gazebo without its window, RViz still shows the robot.
- **The behaviour tree waits forever:** the robot skills start about 20 s after the simulation; wait for
  `kitchen skills ready` in terminal 1.
- **Processes from an earlier run are still alive:** close all terminals of the previous run (or stop
  them with Ctrl+C) before starting a new one.

## Repository layout

| Package | Contents |
|---------|----------|
| `src/kitchen_gazebo` | Gazebo world, launch files, Nav2 configuration and map, objects, RViz configs, and the scripts: robot skills server (`kitchen_skills.py`), PR2 kinematics for aiming the head (`pr2_ik.py`), generators for the Gazebo models, map and Nav2 parameters |
| `src/kitchen_bt` | behaviour tree for setting and clearing the table (`behavior_trees/kitchen_tasks.xml`) and its runner, which writes the explanation log |
| `src/kitchen_interfaces` | `Skill.srv`: one robot skill call (skill name + JSON parameters → success, reason, JSON result) |
| `src/pr2_description` | the lab's PR2 model (from PyCRAM); meshes from `PR2/pr2_common` (BSD) |
| `src/iai_kitchen` | the lab's IAI kitchen model (from PyCRAM); meshes from `code-iai/iai_maps` |
| `docs/` | recordings and pictures of the original lab and the simulation |

Generated files are already in the repository. After changing a generator script, run it again and
rebuild, e.g. `python3 src/kitchen_gazebo/scripts/make_gazebo_urdfs.py && colcon build`.

## Notes

- **Robot:** the project does not have to use the exact PR2 — any robot with the same functionality
  (mobile base, a camera it can aim, a way to carry the objects) is fine. The simulation uses the
  lab's PR2. If the robot is changed, the kitchen, objects, behaviour tree and explanation layer stay;
  the robot model, the head aiming and the Nav2 footprint change.
- **Differences to the original lab:** the robot starts at (0.2, 0.6) instead of (1, 2), where its
  base would be inside the oven; the objects stand next to the sink instead of above its basin; the
  goals on the island are spread so the objects do not touch; the robot stands at the island's edge
  instead of inside it. Instead of grasping, the robot carries each object on a tray (see Carrying).
- **Licenses:** the kitchen meshes come from `code-iai/iai_maps`, which declares no license; keep this
  in mind before making the repository public.
