# GiNGR Framework - Under construction
GiNGR: Generalized Iterative Non-Rigid Point Cloud and Surface Registration Using Gaussian Process Regression. 

The GiNGR framework allows you to perform non-rigid registration with an iterative algorithm that makes use of Gaussian Process Regression in each iteration to find its next state. 

Existing algorithms can be converted into the GiNGR framework, and compared based on 3 properties:
 - Kernel function: how similar should the deformation of neighbouring points be - this is determined based on their correlation
 - Correspondence estimation function: how to estimate corresponding points between the moving instance (reference) and the target.
 - Observation uncertainty: what is the noise assumption of the correspondence estimations?

This framework contains a general library to input these 3 properties. 

## Kernel function
`Add description`
## Correspondence Estimation
`Add description`
## Observation Uncertainty
`Add description`

# Implementation of existing algorithms
The framework contains implementation of the following existing algorithms under the GiNGR framework + the simple implementations of the algorithms themselves for easy comparison.

## CPD: Coherent Point Drift (only Naïve version)
Implementation of the non-rigid CPD algorithm from https://arxiv.org/pdf/0905.2635.pdf

## Optimal Step ICP (N-ICP-T and N-ICP-A):
Implementation of the non-rigid ICP algorithms from https://gravis.dmi.unibas.ch/publications/2007/CVPR07_Amberg.pdf

# Todo
## GiNGR
 - [ ] Usage of landmarks 
    - [ ] pre-alignment
    - [ ] fitting of posterior model
    - [ ] include landmarks in correspondence pairs)
 - [ ] Multiresolution fitting
 - [ ] BCPD implementation
 - [ ] General D implementation instead of _3D
 - [ ] General topology type instead of TriangleMesh
 - [ ] Allow for global rigid alignment in each step (BCPD style)
 - [ ] Efficient P computation for CPD
 - [ ] Logger to enable/disable fitting info
 - [ ] Scalismo-UI hook - to visually see the update steps
 - [ ] Update framework to allow for probabilitic steps (with MH)
    - [ ] General correspondence proposal
 
 
## Bugs
 - [ ] Iterator in GiNGR hard set to max 100