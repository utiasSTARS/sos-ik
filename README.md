# sos-ik
Inverse kinematics using sum of squares optimization.

<img src="https://raw.githubusercontent.com/utiasSTARS/sos-ik/master/system.pdf" width="500px"/>

## Installation and Dependencies 

All code and experiments were developed and run in MATLAB R2018b.

We use the [Sparse-BSOS](https://github.com/tweisser/Sparse_BSOS) Matlab package, which ships with its own modified version of SDPT3 (see the linked repository for setup details).

### Usage
The scripts `examples/ik_solution_extraction_experiment_2d.m` and `examples/ik_solution_extraction_experiment_2d.m` demonstrate usage of our main function `utils/SBSOS/solveIKSBSOS.m` on simulated 2D and 3D problem instances. The default settings of these scripts carry out experiments identical to those in our paper (see citation below). 

## Citation
If you use any of this code in your work, please cite the [relevant publication](https://arxiv.org/pdf/1909.09318.pdf): 

```bibtex
@article{maric2019inverse,
  title={Inverse Kinematics for Serial Kinematic Chains via Sum of Squares Optimization},
  author={Maric, Filip and Giamou, Matthew and Khoubyarian, Soroush and Petrovic, Ivan and Kelly, Jonathan},
  journal={arXiv preprint arXiv:1909.09318},
  year={2019}
}
```
