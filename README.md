## Vortex-Lattice-method to study the aspect ratio of Flat Finite wing.

In this project we are interested to find out how the aspect ratio is going to effect the lift and drag coefficients, which have direct influence on the performance of the aircraft.
For this Flat finite wing, we are using vortex lattice method, to observe the distribution of aerodynamic forces such as lift and drag. Unlike the 2D geometry for a 3D geometry we need to also add the effect 
of induced drag. So VLM is a method by which we will be including the effect of induced drag in the form of wake vortices and on the other hand, to see the contribution of vortex strength
on the lift of the flat wing we also have a Bound vortex.

Analytical solutions for thes models are very difficult hence, we go for numerical simulations based on few assumptions to generate solutions similar to the closed form solutions. Some of the approaches
are as follows.

# Lifting line model:

For 3D geometry finding the analytical solution of the integral equations is tedious. So, Lifting line theory is a procedure used for calculating the lifting properties of a wing by a single lifting line
approximating that will allow the closedform solution. It captures the basic features of 3D lifting flows and predicts the reduction in the lift slope and increase in the induced drag with reducing aspect ratio.

According to Hemholtz theorem, vorticity cannot start or end in the fluid, i.e, the vortex lines do not terminate but they change their direction towards the wake region with 
constant vortex strength. Since there is vorticity produced on top of the wing, these need to be shedded into the wake region. In real flows, the vorticity
is spread across the fluid domain due to the presence of viscocity. However, we are dealing with potential flows the vortex shedding effect is observed to be negligible after the distance of
approximately, 20 * span. 

This lifting line model is kept at the quatercord distance from the leading edge to satisfy the kutta condition. 


# Assumptions:
1. Flow is steady, hence the effect of starting vortex at the end of the wake is negligible.
2. Potential flow
3. From Prandtl lifting line model; sum of the normal velocity components induced by the Wing, wake vortices and normal velocity component of 
the free stream should be equal to zero.
4. Thin aerofoils, the angle of attack should also be very small, so sin(\alpha) \approx \alpha.
5. Boundary condition needs to be satisfied is "Zero normal flow across the body".

# Procedure for constructing a numerical solution:

1. Choice of singularity element: We have choosen a Horse shoe vortex element, there can be many HSE divided on the wing on different pannels (lumped model) or even a single Lifting line HSE
    will suffice, In our case, we have divided the whole wing into n number of pannels having horse shoe elements and the bound vortex for all these are kept on the quater chord line
    collocation point is at the center of the panel's 3/4 chord line. Strength of vortex is constant and since its a lumped model which accounts kutta condition, hence we can also satisfy the kutta condition.
2. Velocity due to the each of the horse shoe element and vortex is calculated from the procedure obtained through biot sarvats law.
3. Discretization and Grid Generation: For flat plate then the normal nj is a function of the local angle αj
4. Calculating the Influence Coefficients
5. Establishing RHS vector
6. Solve linear set of equations
7. Secondary Computation Pressure, loads and velocities


Compare the results with XFLR5 results. 

* For Further Understanding of the method visit, Low Speed Aerodynamics by Katz and Plotkins *
