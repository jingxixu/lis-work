# sample_gp.py
- Sample functions from gaussian process with squared exponential kernel. 
- The true function we try to approximate is f = 2.5 * sin(0.9 * x). 
- The observed data are added with a noise of mean 0 and variance 0.1

## Usage
Tested on `Python 3.6.4`

Run the following commands to see help information
```
python sample_gp.py -h
```
or
```
python sample_gp.py --help
```

To sample functions, run
```
python sample_gp.py --n_func=10 --lengthscale=1 --variance=1 --noise_variance=0
```
Those values shown above are default values; thus, the above command is equivalent to 
```
python sample_gp.py
```

Then three plots will be generated and saved
- The prior functions.
![](https://github.com/jingxixu/lis-work/blob/master/gaussian_process/imgs/prior.png)

- The posterior functions, 3-sigma confidence level, and observed noisy data points.
![](https://github.com/jingxixu/lis-work/blob/master/gaussian_process/imgs/posterior.png)

- The true underlying function, the observed noisy data and the predicted mean with 3-sigma confidence level.
![](https://github.com/jingxixu/lis-work/blob/master/gaussian_process/imgs/true_func.png)

## Algorithm 
See [Gaussian Process for Machine Learning (Algorithm 2.1)](http://www.gaussianprocess.org/gpml/chapters/RW.pdf)
