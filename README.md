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

\begin{algorithm}[t]
	\SetAlgoLined
	\KwIn{$\Gamma_T$ target mesh, \\\hspace{3.0em}GPMM based on the reference mesh $\Gamma_R$}
	\KwOut{$\hat{\Gamma}_R$ based on the best or final $\bm{\alpha}$ state\;}
	\KwData{
	set GPMM parameter, e.g. $\bm{\alpha}^0=\bm{0}$\;
	\hspace{3.0em}initialize correspondence uncertainty $\sigma^2$\;
		}
    \While{While (Deterministic: not converged, Probabilistic: max iteration not reached)}{
        Estimate correspondence deformations $\hat{U}$\;
		Perform GPR, i.e. compute $\mathcal{GP}_p$\;
        \eIf{Stochastic}
        {
            Draw random proposal $\bm{\alpha}'$ from $\mathcal{GP}_p$\;
            \eIf{Accepted}
            {
                Update current state: $\bm{\alpha}^{i+1} = \bm{\alpha}'$\;
            }
            {
                Stay in previous state: $\bm{\alpha}^{i+1} = \bm{\alpha}^{i}$\;
            }
        }{
            Set $\bm{\alpha}^{i+1}$ to mean from $\mathcal{GP}_p$\;
        }
		Update correspondence uncertainty $\sigma^2$\;
	}
	\caption{GiNGR registration}
	\label{algo:GiNGR-steps}
\end{algorithm}