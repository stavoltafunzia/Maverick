# Maverick

Maverick is a library to solve nonlinear Optimal Control Problems (OCP). It is based on a direct, full collocation method, i.e. it converts the optimal control problem into an NLP and then uses a third party NLP solver.
Its distinguishing feature from the large majority of OCP software is the **capability to handle implicit Differential (of the first order)-Algebraic Equations (DAE) and control derivatives**, leading to a much easier formulation. It implements an heuristic mesh refinement algorithm that allows to automatically increase the number of discretisation points until the integration of the DAE equations satisfies a desired accuracy.
Maverick is entirely written in C++ and makes use only of free/open-source third party libraries, thus it can be used on any system endowed with a C++ compiler. I personally tested it also on a Raspberry Pi 3.
The project includes a [Maple](http://maplesoft.com/products/maple/) library that allows to easily declare the optimal control problem within a symbolic algebra environment without writing a single line of C++ code: the C++ code is automatically generated by the Maple library.
The solver workflow and the numeric dataset can be controlled/specified using Lua text files or from within Python, thanks to the Python interface.

## Features

Some of the features of Maverick are the following:
- it allows to use a very general optimal control problem formulation. The formulation accepted by Maverick can be found in formula 6.5 of my [PhD thesis](http://paduaresearch.cab.unipd.it/11053);
- it allows to use implicit first order differential-algebraic equations;
- it allows to use both differential or algebraic states and controls; Derivatives of differential controls can be used in the problem formulation;
- it allows to define general path constraints and integral constraints;
- it converts the NLP multipliers to OCP Lagrange multipliers;
- **it does not require any commercial software**, like MATLAB;
- it exposes user-friendly scaling options to scale the optimal control problem.

The integration scheme currently implemented is based on a midpoint quadrature rule, even if other integration scheme are planned to be implemented in the future.
More information about Maverick implementation can be found in chapter 6 of my [PhD thesis](http://paduaresearch.cab.unipd.it/11053).

## How to compile and install
In order to compile the core C++ library, first install the third party software (see paragraph below). Then simply go into the "lib" folder and type
  > make

to compile, and
  > make install

to install the library in `/usr/local/maverick/`. When compiling the C++ library, you may need to specify some dependency libraries (ipopt and blas) with the command
  >make IPOPT_INCLUDE=\<compiler_flag_to_use_ipopt_headers> IPOPT_LIB=\<compiler_flag_to_link_to_ipopt_library> BLAS=\<compiler_flag_to_link_to_blas_library>

By default ipopt is sought in /usr/local/ipopt, and the blas library used is ATLAS.

If you want to use the Python(3) library, type
  >make python

to compile it, and
  >make install_python

to install it.

If you want to use the Maple library, copy the content of the `maple/Maverick` folder in `<maple_install_dir>/toolbox/Maverick`.

## How to use maverick
In order to solve an optimal control problem problem, the user must (1) define the OCP problem (2) specify the dataset and (3) run the problem.

1) An optimal control problem is defined by a C++ class inheriting from the abstract "MaverickOCP" class. All the pure virtual methods must be implemented. These pure virtual methods give information about the problem, such as: states, controls, equations, path constraints, Lagrange and Mayer target, post processing, Jacobians and Hessians. If your are not familiar with this terminology you can find it explained in classic textbooks such as "Dynamic optimization" (A. E. Bryson), "Practical methods for optimal control and estimation using nonlinear
programming" (J. T. Betts), or in my [PhD thesis](http://paduaresearch.cab.unipd.it/11053).

2) The problem dataset, including numeric values for the optimal control problem parameters, the desired mesh and the solver settings, can be specified in a Lua text files or through the Python interface.

3) The problem can be executed in two different ways.

  * 3.a) The first option is to write a small main program in C++ (less than ten code lines) that basically does the following things:
    * reads the Lua file and loads it into a GenericContainer;
      >lua.do_file('lua_file_name');
      >lua.global_to_GC("Data",gc);

    * creates an instance of the problem class;
      >MyOcpProblem ocp_problem;

    * takes a reference (managed pointer) to an instance of the solver class;
      >unique_ptr\<Maverick::MaverickOcpSolver> solver = Maverick::getMaverickOcpSolver(ocp);

    * tells the solver to solve the problem;
      >solver->solve(gc);

    You can look at the C++ main file in the "source" directory of each example included in the project.

  * 3.b) The second option is to use the Python interface. With the Python interface, the user can specify the dataset in a Python dictionary (data), or alternatively the dataset can be loaded from a Lua file into a Python dictionary. The solver can be invoked from Python and the solution is returned as a Python dictionary with the following simple commands:
    >import sys;

    >sys.path.append('/usr/local/maverick/python')

    >import maverick

    >Data = ..... (problem dataset)

    >solver = maverick.Solver('problem_name', 'path_to_the_problem_source_files')

    >output = solver.solve(Data)

    NOTE: python3 must be used.


Both for 3.a and 3.b please look at the examples for more details.

## Third party software

The following software is required in order to use Maverick:

- an NLP solver. Currently Ipopt (free) is supported, Whorp is planned to be supported. You need to download and compile Ipopt before to build Maverick.

- BLAS. BLAS library actually is not required by Maverick itself but by Ipopt. A BLAS library must be supplied when compiling the linking library between Maverick and Ipopt included in the project.

- Lua. Lua can be used to declare the problem dataset in a lua file that is then processed by Maverick. A Lua version (5.3.3) is already included in the project. If you want to use your own lua library, you simply have to declare the headers and the library when building the project:
  >make LUA_INCLUDE=\<compiler_flag_to_use_lua_headers> LUA_LIB=\<compiler_flag_to_link_to_lua_library>

- [Ebertolazzi GenericContainer](http://github.com/ebertolazzi/GenericContainer). This open source library is already included (in a slightly modified version) in the project. No download needed.

- [Ebertolazzi Splines](http://github.com/ebertolazzi/Splines). This open source library is already included (in a slightly modified version) in the project. No download needed.

- [Eigen](http://eigen.tuxfamily.org). The linear algebra library Eigen is already included in the project. No download required.

- (optional) [Maple](http://maplesoft.com/products/maple/). The Maple symbolic algebra software can be used to easily declare the OC problem within a user-friendly interface. The C++ code of the OCP can be automatically generated by the included Maple library.

- (optional) [MBSymba](http://multibody.net/mbsymba/). MBSymba is a Maple library that allows to easily write the equations of motion for complex multibody systems. It is required only to execute some Maple worksheet in the example folder.

Moreover a C++ and a Fortran compiler are needed. Tested with g++ and gfortran.

## Examples
Each example is organised in four folders: "data", "maple", "results" and "sources". The "data" folder contains the Lua text files with the problem dataset, or the python files. The "maple" folder contains the Maple worksheet to generate the C++ code of the examples. If you do not have Maple, you can neglect this folder since C++ source codes are already included. The "results" folder contains the solution text files. The "sources" folder contains the C++ source files for the problem.

In order to run an example, you must:

1) (Optional) execute the Maple worksheet in the "maple" folder to generate the C++ files in the "sources" folder. If you do not have Maple you can skip this step.

2) From within the "sources" folder, execute "compile.sh" to compile the problem C++ code. This generate a shared library and an executable. Both have the name of the problem, moreover the executable has a suffix ".macosx" or ".linux" depending on the platform you are using.

3) Run the example:
  * 3.a) option A: execute the problem reading the Lua datafile. From within the "sources" folder, run the executable by passing the lua datafile as first argument, i.e. "./problem.macosx ../data/data.lua". The solution text file will be written in the "results" folder.

  * 3.b) option B: execute the problem from within Python. From within the "data" folder, type "python run.py". In this way, the problem dataset is declared in the python file, then the problem shared library is loaded and the problem is solved. The solution data is returned to python in the form of a dictionary.

## Project Status
This software is the result of few months of work during my PhD. Originally this software was intended only for a limited use restricted to my research group; only at the end of the work I decided to publish it in GitHub.
Therefore the code is not thoroughly commented, in some parts it may appear a little messed up or inconsistent (the latter means I may have used different programming choices to implement the same functionality).
However, it works: you can download it, compile and run. I'm currently using it for work.
A "cleaning" of the code would make the code much more readable and easier to understand for other developers. Currently I do not have enough time to dedicate to the reviewing all the code, and at the same time to implement further functionalities.
If someone is interested in this project and would like to help me in carrying on the development, please contact me. Even if you know a little of Optimal Control but you are a good programmer, your help can be very useful.

## FAQ

- **Which operating system is supported?**

Maverick is written entirely in plain C++ and no OS specific api is used: thus it can be used in any OS provided you have a C++ compiler. However, I'v tested and used Maverick only in unix systems (not in Windows).
In theory you can compile Maverick also in Windows, however you cannot use the makefile included. It is not my intention to support Windows since I personally do not want to waste time using such (quasi-pseudo) operating system. If you are using Windows, you can install a virtual machine with Linux; performance shouldn't be affected remarkably.
Ps: if you would like to contribute to the project adding Windows support, please contact me.

- **How should I write Jacobians and Hessians?**

The Jacobian matrix (of a vector function F of a vector variable X) at the i-th row and j-th column has the derivative of the i-th function component F_i taken with respect to the j-th variable component X_j.
The Hessian matrix (of a scalar function f of a vector variable X) at the i-th row and j-th column has the second order derivative of the function f taken with respect to the i-th variable component X_i and the j-th variable component X_j.
You must write the Jacobian and Hessian matrices in a plain 1-dimensional array using column-major ordering. Only the nonzero entries (see note below) of such matrices must be written, i.e. if a matrix as N nonzero entries, then the 1-dimensional array must have length N. The matrix position (pattern) of each array element must be returned in the "pattern" MaverickOCP methods.
NOTE: the nonzero matrix elements are those elements that CAN be nonzero for some values of the vector variable X.

- **I do not want to provide Jacobians and Hessians. How can I do?**

The best choice is to use the Maple library to declare the optimal control problem (see examples): in this way, Jacobians and Hessians are automatically generated. This is the most efficient and robust solution that can be used, and it is highly recommended.
Alternatively, you may use finite difference approximation or automatic differentiation. However such features are not implemented yet, even if I've planned to implement them in a close future.

- **I do not understand how to implement the C++ class MaverickOCP**

Please have a look at the examples. Everything can be understood by looking at the examples.

- **The difference between states (or controls) and algebraic states (or controls) is not completely clear to me**

States (or controls), also named differential states (or controls), are those states (or controls) whose derivatives take part in the OCP definition, e.g. their derivative appear in the equations of motion or in the Lagrange target. Algebraic states (or controls) are those states (or controls) whose derivatives do not appear in the problem. This distinction is required for numerical reasons, as described in my [PhD thesis](http://paduaresearch.cab.unipd.it/11053).

- **The NLP solver does not manage to find the solution**

First of all, keep in mind that the problem you are trying to solve could be infeasible or undetermined. If this is the case, the solution does not exists. Even if this is not the case, the problem can be ill-conditioned, so it is very difficult to find a solution numerically. Scaling the problem is fundamental in every optimal control problem, so you should try to scale the optimal control problem playing with the "Scaling" options that Maverick exposes. Finally, even if the problem is not ill-conditioned, it can be difficult to solve if you are using algebraic equations of high DAE index.

## Contacts
If you want to contact me you can email to <my_name>.<my_surname> at gmail.com

## License
This software is released under the GNU General Public License v3.0.
