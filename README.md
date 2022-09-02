# L1-adaptive-control
According to the state of the art, the **L1-adaptive controllers are among the fastest adapting controllers**. 
These high adaptation rates allow a **prediction of the transient performance** of the closed loop system as well as the **performance in steady state**. 
In addition **robustness limits** can be determined with respect to the **controlled system parameters and dead-times**. 

This **eliminates many of the disadvantages of slow adapting controllers**, such as MRAC:
- Parameter drift in the case of too little excitation, resulting in bursting, a temporary instability of the entire control loop.
- instability of the control loop at unmodeled dynamics

The **wide range of application areas** such as flight controls (unmanned aircraft, spacecraft, helicopters, missiles), autopilots for boats, and robotic and process automation systems indicate the effectiveness and excellent variability of L1 adaptive control. 


Here are some good paper about the L1-adaptive control theory:
- https://engineering.purdue.edu/~zak/ECE675__2022/L1_adaptive_control.pdf.
- https://www.researchgate.net/publication/224640323_Design_and_Analysis_of_a_Novel_L1_Adaptive_Controller_Part_I_Control_Signal_and_Asymptotic_Stability
- https://www.researchgate.net/publication/251820946_Design_and_Analysis_of_a_Novel_L1_Adaptive_Controller_Part_II_Guaranteed_Transient_Performance
- https://www.researchgate.net/publication/228756594_L1_Adaptive_Controller_for_a_Missile_Longitudinal_Autopilot_Design

Recently it has also been used to **robustify a reinforcement learning agent** to control a double pendulum:
- https://arxiv.org/pdf/2112.01953.pdf
- https://www.youtube.com/watch?v=xZBcsNMYK3Y

Further work of my Master thesis supervisor and my professor:
- https://www.db-thueringen.de/servlets/MCRFileNodeServlet/dbt_derivate_00041143/ilm1-2017000665.pdf
- https://www.imms.de/referenzen/publikationen/d/robust-tracking-control-with-l1-adaptive-augmentation-for-a-long-stroke-vertical-nanopositioning-system-part-i-4666.html

Another very fast adapting controller is the **Simple Adaptive Controller (SAC)** by Itzhak Barkana. A very good paper about it can be found here:
- https://www.researchgate.net/publication/269401168_Adaptive_Control_But_is_so_Simple_A_Tribute_to_the_Efficiency_Simplicity_and_Beauty_of_Adaptive_Control

Now, here I want to present my master thesis and some Matlab/Simulink files for everybody who is interested in the L1-adaptive control theory.
I hope, that this helps you to design some awesome adaptive controller!


## Robust L1-adaptive control of a planar nanopositioning system under model uncertainties and disturbances (Master Thesis)
### Abstract
This master thesis deals with the investigation, selection and design of suitable adaptive control methods
for a planar nanopositioning system. For this purpose, controllers of the L1-adaptive control theory, a
novel theory for adaptive control, are considered and compared with conventional adaptive controllers,
such as the MRAC. The disturbances and uncertainties acting on the nanopositioning system are
described and an L1-adaptive state feedback and an L1-adaptive output feedback controller are
designed. Finally, these are enhanced to develop a dynamic trajectory tracking controller.

**Link:** https://github.com/mpappert/L1-adaptive-control/blob/main/L1-adaptive-control-of-nanopositioning-system.pdf
