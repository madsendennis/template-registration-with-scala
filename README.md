# Point set registration
This library contains different template matching algorithms that works either on meshes or on pure pointclouds. 
Some of the algorithms require one to specify a deformation model in form of a Gaussian Process Morphable Model.

Until now, only the non-rigid CPD algorithm is implemented
# Coherent Point Drift implementation in Scala (Still in an initial phase):
Implementation of the non-rigid CPD algorithm from https://arxiv.org/pdf/0905.2635.pdf
Part of the implementation is directly taken from the python implementation of CPD https://github.com/siavashk/pycpd.
