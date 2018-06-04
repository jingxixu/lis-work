import numpy as np
import matplotlib.pyplot as plt
import argparse
np.set_printoptions(suppress=True)

def kernel(X1, X2):
	'''
	Calculate the covariance matrix
	args:
		X1: n by d matrix, each row is a d-dimensional data point
		X2: m by d matrix, each row is a d-dimensional data point
	return: 
		K(X1, X2): n by m covariance matrix
	'''
	lengthscale=args.lengthscale
	variance=args.variance
	# cool vectorization. note how (n, 1) is braodcast with (m, )
	squredist = np.sum(X1**2, axis=1).reshape(-1, 1) + np.sum(X2**2, axis=1) - 2*np.dot(X1, X2.T)
	return variance**2 * np.exp(-1/(2*lengthscale) * squredist)

def sample_from_gp(n_func=5, noise_variance=0.):
	'''
	Sample functions from gaussian process. 
	Args:
		n_func: number of functions to sample
		noise_variance: amount of variance the algorithm assumes on the data
	It shows and saves three plots:
		- prior functions
		- posterior functions with 3-sigma confidence level and observed data
		- obervered data, true function and mean
	'''
	# this is the true unknown function we are trying to approximate
	f = lambda x: 2.5*np.sin(0.9*x).flatten()
	np.random.seed(100)
	# data to plot the true function
	n_plot = 1000
	X_plot = np.linspace(-5, 5, n_plot)
	y_plot = f(X_plot)
	# noisy training/observed data points
	n_train = 10
	X_train = np.random.uniform(-5, 5, n_train).reshape(-1, 1)		
	y_train = f(X_train) + np.random.randn(n_train) * 0.1	
	# test data points to sample a function from the GP
	n_test = 100
	X_test = np.linspace(-5, 5, n_test).reshape(-1, 1)	

	K = kernel(X_train, X_train)	
	L = np.linalg.cholesky(K + np.eye(n_train)*noise_variance)	
	alpha = np.linalg.solve(L.T, np.linalg.solve(L, y_train))

	# compute mean at test points
	Ks = kernel(X_train, X_test)
	mu = np.dot(Ks.T, alpha)

	# compute covariance at test points
	# prior covariance
	Kss = kernel(X_test, X_test)
	V = np.linalg.solve(L, Ks)
	# posterior covariance
	Cov = Kss - np.dot(V.T, V)
	# compute variance at test points
	# another cool vectorization
	# np.diag(AB) = np.sum(A*B.T, axis=1)
	var = np.diag(Kss) - np.sum(V.T**2, axis=1)
	standev = np.sqrt(var)

	# sample functions from the prior
	L = np.linalg.cholesky(Kss + 1e-6*np.eye(n_test))
	f_prior = np.dot(L, np.random.randn(n_test, n_func))
	fig, ax = plt.subplots(figsize=(12, 8))
	plt.axis([-5, 5, -3, 3])
	plt.plot(X_test, f_prior, '-')
	plt.title('prior')
	plt.savefig('prior.png', bbox_inches='tight')

	# sample functions from the posterior
	L = np.linalg.cholesky(Cov + 1e-6*np.eye(n_test))
	f_post = mu.reshape(-1, 1) + np.dot(L, np.random.randn(n_test, n_func))
	fig, ax = plt.subplots(figsize=(12, 8))
	plt.axis([-5, 5, -3, 3])
	plt.plot(X_test, f_post, '-')
	plt.plot(X_train, y_train, '+', color='r', markersize=17, label='y (traing data)')
	plt.gca().fill_between(X_test.flat, mu-3*standev, mu+3*standev, color="#dddddd")
	plt.title('posterior')
	plt.savefig('posterior.png', bbox_inches='tight')

	# plot the true function and mean
	fig, ax = plt.subplots(figsize=(12, 8))
	plt.axis([-5, 5, -3, 3])
	plt.plot(X_train, y_train, '+', color='r', markersize=17, label='y (traing/observed data)')
	plt.plot(X_test, mu, '-', label='predicted mean')
	plt.plot(X_plot, y_plot, '-', color='r', label='f (true function)')
	plt.legend(loc='upper right')
	plt.gca().fill_between(X_test.flat, mu-3*standev, mu+3*standev, color="#dddddd")
	plt.title('true function and predicted mean')
	plt.savefig('true_func.png', bbox_inches='tight')
	plt.show()

def main():
	sample_from_gp(n_func=args.n_func, noise_variance=args.noise_variance)

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Sample functions from gaussian process')
	parser.add_argument('--n_func', type=int, default=10, help='number of functions to sample')
	parser.add_argument('--lengthscale', type=float, default=1., help='lengthscale parameter of squared exponential kernel')
	parser.add_argument('--variance', type=float, default=1., help='variance parameter of squared exponential kernel')
	parser.add_argument('--noise_variance', type=float, default=0., help='variance of noise assumed on observed data')
	args = parser.parse_args()
	main()
	