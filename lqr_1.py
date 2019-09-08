# Weekly Code           #2
# LQR                   Version 1.0.0       Date: 9/7/2019
# solving a inverted pendulum using lqr ref. below the same using matlab
# http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
# Future Improvements:  1) add visualization
#                       2) animation and simulation
#                       3) comparison with various Q and R values.



import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg



M = 0.5
m = 0.3
b = 0.1
I = 0.006
g = 9.8
l = 0.3

p = I*(M+m) + M*m*l**2
# state space representation
A = np.array([[0, 1, 0, 0],
              [0, -(I+m*l**2)*b/p, (m**2*g*l**2)/p, 0],
              [0, 0, 0, 1],
              [0, -(m*l*b)/p, m*g*l*(M+m)/p, 0]])
B = np.array([[0],
     [(I+m*l**2)/p],
     [0],
     [m*l/p]])
C = np.array([[1, 0, 0, 0],
     [0, 0, 1, 0]])
D = np.array([[0],
     [0]])
A_size = A.size/2
# print("The poles of A are: ")
print(A)
# [w, v] = np.linalg.eig(A)
# print(w)
# We see that the system is unstable, which is expected.
# We check for controllability
AB = np.matmul(A, B)
A2B = np.matmul(A, AB)
A3B = np.matmul(A, A2B)
A4B = np.matmul(A, A3B)

# forming the controllability matrix
co = np.zeros((4, 4), float)
# filling the controllability matrix
for i in range(4):
    co[i, 0] = B[i]
    co[i, 1] = AB[i]
    co[i, 2] = A2B[i]
    co[i, 3] = A3B[i]
# print(co)
print(" The rank of ctrb = {}".format(np.linalg.matrix_rank(co)))
# we see system is controllable

# we select the R and Q matrices as follows
Q = np.matmul(C.T, C)
R = np.array([[1]])

# now we solve the countinous time ricatti equation
X = np.array(linalg.solve_continuous_are(A, B, Q, R))
print(X)
# computing the LQR Gain
K = np.linalg.inv(R)@(B.T@X)
print("K = {}".format(K))
# computing eigen values
[w, v] = np.linalg.eig(A-B@K)
print("The eigen values of the closed loop systems are \n {}".format(w))
# We see that all eigen value are in LHP
# we check from MATLAB the answer we recieve matches.
