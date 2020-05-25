# sos-ik
Inverse kinematics using sum of squares optimization.

<img src="https://raw.githubusercontent.com/utiasSTARS/sos-ik/master/system.png" width="500px"/>

## Installation and Dependencies 

All code and experiments were developed and run in MATLAB R2018b.

We use the [Sparse-BSOS](https://github.com/tweisser/Sparse_BSOS) MATLAB package, which ships with [its own modified version of SDPT3](https://github.com/tweisser/Sparse_BSOS/tree/master/SDPT3-4.0_modified_by_KCT). In order to use our package, you must replace the version of the file `sdpt3.m` found in that package with the one in this repository. You must also replace `certificates/auxfun/csol.m` withou the version of `csol.m` found in this respository. Both of these files are in the [`replacement_files/`](https://github.com/utiasSTARS/sos-ik/tree/master/replacement_files) directory.

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
