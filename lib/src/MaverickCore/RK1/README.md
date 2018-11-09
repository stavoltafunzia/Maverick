This folder contains numeric classes (ocp2nlp discretiser, integrator, mesh, mesh refiner) related to the Runge-Kutta 1-st order integration scheme.
The implemented 1-st order Runge Kutta scheme is described by the following Butcher tableau:
```
alpha | alpha
      |   1
```
Where alpha is comprised between 0 and 1 (included) and is a parameter of all classes.
The explicit Euler, implicit Euler and implicit midpoint integration schemes can be obtained respectively with alpha = 0, alpha = 1 and alpha = 0.5.
