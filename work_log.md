## Week 0
__May 28th - June 1st__ 
- [X] use an existing domain file from [pddlstream](https://github.com/caelan/pddlstream) to solve Sussman anomaly ([link](https://github.com/jingxixu/lis-work/tree/master/pddl_examples/sussman_anomaly))
- [X] create a planning problem of cooking a meal and solve it use the [pddlstream](https://github.com/caelan/pddlstream) ([link](https://github.com/jingxixu/lis-work/tree/master/pddl_examples/cook_meal))
- [X] add a score function to learn the stir action parameters for [Kitchen2D](https://github.com/JingxiXu/Kitchen2D)
  - The score function is still very naive and is purely equivalent to the combined entropy of multiple partitions. The learning of stir action suffers 
  from the fact that the particles start moving and mixing right after they are created. So the score function is changing even before stiring 
  and the score is pretty high initially.
  - It might be better to think about the maximum entropy of a combination of multiple parts. Because if one kind of particle has a dominant 
  proportion, then the score of stir defined above will always be low, but it does not mean we do not stir well.

## Week 1
__June 4th__
- [X] write a script to sample functions from gaussian process ([link](https://github.com/jingxixu/lis-work/tree/master/gaussian_process))

__June 5th__
- [X] finish the paper [Additive Gaussian Process](https://arxiv.org/abs/1112.4394)
- [X] reimplement the above paper's experiment on toy synthetic data and obtain similar results ([link](https://github.com/jingxixu/lis-work/blob/master/additive-gps/sythetic_demo.ipynb))

<img src="https://github.com/jingxixu/lis-work/blob/master/imgs/true.png" height="190"> &emsp;
<img src="https://github.com/jingxixu/lis-work/blob/master/imgs/ard_rbf.png" height="190"> &emsp;
<img src="https://github.com/jingxixu/lis-work/blob/master/imgs/additive_gp_sythetic.png" height="190"> &emsp;

__June 6th__
- [X] with the help of Caelan, make all pybullet examples at [pddlstream](https://github.com/caelan/pddlstream) runnable by fixing breaking commit
- [X] finish [Robot Academy](https://github.com/Learning-and-Intelligent-Systems/lis_pr2_pkg/wiki/Robot-Academy) class 1 ([code](https://github.com/jingxixu/lis-work/blob/master/robot_accademy/dance_jingxi.py), [video](https://youtu.be/N1F5F4eFNtI))

<p align="center">
  <img align="middle" src="https://img.youtube.com/vi/N1F5F4eFNtI/0.jpg" height="180">(https://youtu.be/N1F5F4eFNtI)
</p>
