# GiNGR Framework
GiNGR: Generalized Iterative Non-Rigid Point Cloud and Surface Registration Using Gaussian Process Regression. 

The GiNGR framework allows you to perform non-rigid registration with an iterative algorithm that makes use of Gaussian Process Regression in each iteration to find its next state. 

Existing algorithms can be converted into the GiNGR framework, and compared based on 3 properties:
 - Kernel function: how similar should the deformation of neighbouring points be - this is determined based on their correlation
 - Correspondence estimation function: how to estimate corresponding points between the moving instance (reference) and the target.
 - Observation uncertainty: what is the noise assumption of the correspondence estimations?

This framework contains a general library to input these 3 properties. 

## General use
To use GiNGR, one need to specify the deformation model to use in form of a GPMM model as well as the correspondence estimation function and the uncertainty update.
### Define the prior model
The creation of the GPMM is separate from the registration step. First go to `apps/gpmm/Create-GPMM` and adjust the kernel to use and the kernel parameters. After the GPMM has been computed, a UI will show up where the deformation model can be evaluated by sampling from it.
### Configure the registration algorithm
The next step is to define the correspondence and uncertainty estimation update. 
For this, default configurations have been implemented for CPD and ICP. 
Simple Demo applications can be found in `apps/registration/DemoICP` and `apps/registration/DemoCPD`

First the configuration file have to be adjusted, which controls the hyperparameters for the selected registration method.
Then the `SimpleRegistrator` class can be used to initialize the algorithm and run the registration.
### GiNGR state
In each iteration a new GiNGR state is computed which contains the GPMM model, the current `fit`, the target as well as all the GPMM model parameters (non-rigid and global pose).

### Deterministic vs Probabilistic


The probabilistic implementation is based on the ICP-Proposal repository: https://github.com/unibas-gravis/icp-proposal
#### Visualizing the posterior output from probabilistic fitting
`apps/registration/DemoPosteriorVisualizationFemur`

### Inclusion of Landmarks
`apps/registration/DemoLandmarks`

### Multi-resolution fitting
`apps/registration/DemoMultiResolution`


## Implementation of existing algorithms
In the GiNGR code base, the basic implementations of existing algorithms can also be found for comparison. The algoriths are found under `other/algorithms`
### CPD: Coherent Point Drift (only Naïve version)
Implementation of the CPD algorithm from https://arxiv.org/pdf/0905.2635.pdf
### BCPD: Bayesian Coherent Point Drift (only Naïve version)
Implementation of the BCPD algorithm from https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8985307
### Optimal Step ICP (N-ICP-T and N-ICP-A):
Implementation of the non-rigid ICP algorithms from https://gravis.dmi.unibas.ch/publications/2007/CVPR07_Amberg.pdf

# Todo
 - [ ] Convert 3D to D and use implicit functions for 1D, 2D, 3D