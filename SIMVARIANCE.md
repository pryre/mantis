# Simulation Variance
The following test are configured to be performed in an attempt to model control robustness without changing the tuning.

In each test, the robotic arm makes the following movements:
1. Joint 1: 0.000, Joint 2: 0.000
2. Joint 1: 0.785, Joint 2: 0.000
3. Joint 1: 0.785, Joint 2: 0.785
4. Joint 1: 1.571, Joint 2: 0.000
5. Joint 1: 0.000, Joint 2: 0.000

#### Simulations Run
- Baseline: No model-controller variance
- Arm20d: Multirotor arm length decreased by 20%
- Arm10d: Multirotor arm length decreased by 10%
- Arm10i: Multirotor arm length increased by 10%
- Arm20i: Multirotor arm length increased by 20%
- Thrust20d: Motor Kv rating decreased by 20%
- Thrust10d: Motor Kv rating decreased by 10%
- Thrust10i: Motor Kv rating increased by 10%
- Thrust20i: Motor Kv rating increased by 20%
- MassB20d: Mass of base decreased by 20%
- MassB10d: Mass of base decreased by 10%
- MassB10i: Mass of base increased by 10%
- MassB20i: Mass of base increased by 20%
- MassU20d: Mass of upperarm decreased by 20%
- MassU10d: Mass of upperarm decreased by 10%
- MassU10i: Mass of upperarm increased by 10%
- MassU20i: Mass of upperarm increased by 20%
- MassF20d: Mass of forearm decreased by 20%
- MassF10d: Mass of forearm decreased by 10%
- MassF10i: Mass of forearm increased by 10%
- MassF20i: Mass of forearm increased by 20%
- InertialB20d: Inertial of base decreased by 20%
- InertialB10d: Inertial of base decreased by 10%
- InertialB10i: Inertial of base increased by 10%
- InertialB20i: Inertial of base increased by 20%
- InertialU20d: Inertial of upperarm decreased by 20%
- InertialU10d: Inertial of upperarm decreased by 10%
- InertialU10i: Inertial of upperarm increased by 10%
- InertialU20i: Inertial of upperarm increased by 20%
- InertialF20d: Inertial of forearm decreased by 20%
- InertialF10d: Inertial of forearm decreased by 10%
- InertialF10i: Inertial of forearm increased by 10%
- InertialF20i: Inertial of forearm increased by 20%

