# Point set registration
This library contains different template matching algorithms that works either on meshes or on pure pointclouds. 
Some of the algorithms require one to specify a deformation model in form of a Gaussian Process Morphable Model.

# Coherent Point Drift implementation in Scala (only Naïve version):
Implementation of the non-rigid CPD algorithm from https://arxiv.org/pdf/0905.2635.pdf
Part of the implementation is directly taken from the python implementation of CPD https://github.com/siavashk/pycpd.

Run the demo application: main/apps/demo/NonRigidCPDRegistration

# Optimal Step ICP (N-ICP-T and N-ICP-A):
Implementation of the non-rigid ICP algorithms from https://gravis.dmi.unibas.ch/publications/2007/CVPR07_Amberg.pdf . 

Run the demo application: main/apps/demo/NonRigidOptimalStepICPRegistration
