# quaternion.h

A simple library for quaternions

Alejandro F Queiruga  
UC Berkeley  
c. 2012

## Intro

Everyone hand rolls their own library for quaternions; here's mine. It was an important component to my master's thesis, where it was used for collision detection with 3D polyhedra. These codes were generated with long-lost Mathematica code.

The useful part of note is the equation for rate-of-change of quaternions,
```
dq/dt = 0.5 * (0+w) . q
```
where `w` is the angular velocity vector (and `0+w` is augmenting it with a zero scalar to use quaternion algebra.) There are two versions depending on which frame the angular velocity is measured in---so watch out. The formulation is quite stable when using RK4 and very unstable using forward Euler. I would re-normalize it every timestep anyways. 

## License

LGPL, as per LICENSE.txt
