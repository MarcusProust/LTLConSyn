# LTLControlSynthesis

This is the accompanying code for the paper "Zonotope-based Symbolic Controller Synthesis for Linear Temporal Logic Specifications" by Wei Ren, Raphael M. Jungers, and Dimos V. Dimarogonas. This code is to generate finite local controllers to achieve the global LTL task.

# Preliminaries 
You need to install [CORA](https://tumcps.github.io/CORA/) for the zonotope generation and [SCOTS](https://gitlab.lrz.de/hcs/scots) for the abstraction construction. 

# How to run
1. Run [Zonotope_Covering.m] to obtain the zonotope-based covering
2. Run [Abstract_Controller_Desgin.cc] to generate all local controllers 
3. Run [Position_Trajectory.m] to derive the position trajectory
