import numpy as np

target = np.array([ 2329.98740018, 18782.88246936,  8961.86067478]) # Target position in km

y = np.array([20677.48801802, 24895.98298007,
                16465.70462416, 26507.04323936, 22212.37447646]) # Distances to satellites in km

satellite_positions = np.array([[15600,  7540, 20140],
                                [18760,  2750, 18610],
                                [17610, 14630, 13480],
                                [19170,   610, 18390],
                                [17800,  6400, 19000]]) # Satellite positions in km


def calculate_distance(point1, point2):
    '''Calculate the Euclidean distance between two 3D points.'''
    return np.sqrt((point1[0] - point2[0])**2 + 
                   (point1[1] - point2[1])**2 + 
                   (point1[2] - point2[2])**2)


var_Y = 10  # variance of measurement noise
Sigma_y = np.eye(len(satellite_positions)) * var_Y  # noise covariance matrix
inv_Sigma_y = np.linalg.inv(Sigma_y)  # inverse of noise covariance matrix

x_init = np.zeros(3)  # initial guess for target position

N = 10  # number of iterations
dx_i = np.zeros((N, 3)) # change in position at each iteration
x_i = np.zeros((N+1, 3)) # estimated position at each iteration
x_i[0] = x_init # set initial guess

for i in range(N):
    y_comp = np.array([calculate_distance(satellite, x_i[i]) for satellite in satellite_positions])
    dy = y - y_comp
    J = np.array([(x_i[i] - satellite) / calculate_distance(satellite, x_i[i]) for satellite in satellite_positions])
    dx_i[i] = np.linalg.inv(J.T @ inv_Sigma_y @ J) @ (J.T @ inv_Sigma_y @ dy)
    x_i[i+1] = x_i[i] + dx_i[i]
    
x_hat = x_i[-1] # final estimated position
Sigma_x_hat = np.linalg.inv(J.T @ inv_Sigma_y @ J)
sigma_x_hat = np.sqrt(np.diag(Sigma_x_hat)) # standard deviations of estimated position
print("Estimated position:", x_hat)
print("Uncertainty (1 std dev):", sigma_x_hat)
print("True position:", target)

assert np.all(np.abs(x_hat - target) < 3 * sigma_x_hat), "Estimated position is not within 3 standard deviations of the true position"