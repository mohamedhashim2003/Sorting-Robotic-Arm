# 3DOF Sorting Robotic Arm

## 📖 Overview
This repository contains the mathematical modeling and MATLAB implementation for a 3-DOF planar serial manipulator used in automated sorting operations. It includes comprehensive derivations and code for forward/inverse kinematics, Jacobian matrices, and Lagrangian dynamics.

## 📂 Repository Structure
```text
├── .gitignore
├── Dynamics.m                   # Symbolic derivation of Lagrangian dynamics
├── Forward.m                    # Forward kinematics calculations
├── ForwardGUI.m                 # Graphical user interface for forward kinematics
├── LICENSE                      # Custom Personal & Educational Use License
├── README.md                    # Project documentation
├── complete_Jacobian.m          # Analytic Jacobian matrix computation
├── complete_Kinematics.m        # Master script for full kinematic analysis
├── compute_forces_and_torques.m # Joint torque calculations
├── dynamics_numerical.m         # Numeric verification of the mass matrix and gravity vector
└── inverse.m                    # Inverse kinematics analytical solver
```text

⚙️ Prerequisites
To run the scripts in this repository, you will need:

MATLAB (R2021a or newer recommended)

Symbolic Math Toolbox (Required for the analytic derivations and Christoffel symbols)

🚀 Usage
Clone the repository: git clone https://github.com/mohamedhashim2003/Sorting-Robotic-Arm.git
Open MATLAB and navigate to the cloned repository folder.

Run individual scripts depending on the analysis you want to perform:

Run ForwardGUI.m to launch the interactive kinematics interface.

Run dynamics_numerical.m to verify the mass/inertia matrix and gravity vector numerically.

Run complete_Kinematics.m to output the full kinematic transformations.

Gemini said
You are absolutely right. That is a great catch. A standard GitHub README needs more than just the theory—it needs a repository tree, setup instructions, and prerequisites so anyone landing on the page knows exactly how to navigate and run your files.

Since you shared the screenshot of your repository earlier, I can map out the exact file structure you uploaded. I have also added the standard "Prerequisites" and "Usage" sections.

Here is the fully fleshed-out, typical GitHub README.md template. You can copy the entire block below and paste it directly into your file (just remember to delete the `` tags, as I included them to reference your document!):

Markdown
# 3DOF Sorting Robotic Arm

## 📖 Overview
[cite_start]This repository contains the mathematical modeling and MATLAB implementation for a 3-DOF planar serial manipulator used in automated sorting operations[cite: 1, 5]. [cite_start]It includes comprehensive derivations and code for forward/inverse kinematics, Jacobian matrices, and Lagrangian dynamics[cite: 8].

## 📂 Repository Structure
```text
├── .gitignore
├── Dynamics.m                   # Symbolic derivation of Lagrangian dynamics
├── Forward.m                    # Forward kinematics calculations
├── ForwardGUI.m                 # Graphical user interface for forward kinematics
├── LICENSE                      # Custom Personal & Educational Use License
├── README.md                    # Project documentation
├── complete_Jacobian.m          # Analytic Jacobian matrix computation
├── complete_Kinematics.m        # Master script for full kinematic analysis
├── compute_forces_and_torques.m # Joint torque calculations
├── dynamics_numerical.m         # Numeric verification of the mass matrix and gravity vector
└── inverse.m                    # Inverse kinematics analytical solver

⚙️ Prerequisites
To run the scripts in this repository, you will need:
MATLAB (R2021a or newer recommended)
Symbolic Math Toolbox (Required for the analytic derivations and Christoffel symbols)

🚀 Usage
Clone the repository:
git clone [https://github.com/mohamedhashim2003/Sorting-Robotic-Arm.git](https://github.com/mohamedhashim2003/Sorting-Robotic-Arm.git)
Open MATLAB and navigate to the cloned repository folder.
Run individual scripts depending on the analysis you want to perform:
Run ForwardGUI.m to launch the interactive kinematics interface.
Run dynamics_numerical.m to verify the mass/inertia matrix and gravity vector numerically.
Run complete_Kinematics.m to output the full kinematic transformations.

📐 System Parameters
The physical and mechanical specifications of the robotic arm are modeled as follows:
Base: Fixed cylindrical base with a 12 cm diameter.
Links: 3 homogeneous slender rods. Link 1 is 10 cm, Link 2 is 12 cm, and Link 3 is 12 cm.
Mass: Each link has a mass of 0.05 kg (50 g).
Environment: Planar motion with gravity acting vertically downward in the plane.

🧮 Mathematical Modeling
1. Kinematics
Forward & Inverse: Calculates the end-effector pose using Denavit-Hartenberg (DH) parameters and solves for joint angles given a target pose, accounting for elbow up/down configurations.
Jacobian Matrix: Maps joint velocities directly to end-effector Cartesian velocities.
2. Lagrangian Dynamics
The equations of motion are derived using the Euler-Lagrange formulation:
D(q)q'' + C(q,q')q' + G(q) = tau
D(q): Symmetric mass/inertia matrix.
C(q,q'): Coriolis and centrifugal matrix derived via Christoffel symbols.
G(q): Gravity vector.

📄 License
(See the LICENSE file for details. This code is restricted to personal, internal, and educational use. Commercial use is strictly prohibited without prior written consent.)
