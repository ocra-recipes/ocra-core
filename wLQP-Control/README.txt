
ISIRController
==============

The ISIRController is a software package dedicated to the dynamic control
a robotic systems, especially humanoid, based on a LQP.

This repository is composed of 3 sub-repositories, which must be installed in
this order:

* orc_framework_bin
* quadprog
* orcisir_ISIRController


orc_framework_bin
-----------------

These libraries have been develop by the CEA/LIST, a french robotic laboratory.
The orc framework (meaning "Optimization for Robotic Control") is an interface
to facilitate the translation of the robotic control problem into an
optimization program.


quadprog
--------

This library is a LQP (Linear Quadratic Program) based on the QuadProg++ project
(http://quadprog.sourceforge.net/) which has been slightly modified. In this
version, vector and matrix classes are replaced by Eigen classes, in order to
use the same definitions as the orc framework libraries.


orcisir_ISIRController
----------------------

This library is a c++ package to control robotic systems in dynamics, based on
the thesis of Joseph Salini (http://hal.archives-ouvertes.fr/tel-00710013/).
It lies on the orc framework to define the control problem.
