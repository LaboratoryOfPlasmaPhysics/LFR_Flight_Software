import lfr
import numpy as np

B1=1.+0j
B2=0.+1.j
B3=1.+1.j
E1=3.+1.j
E2=1.+3.j
VECT = np.array([[B1,B2,B3,E1,E2]], dtype=np.csingle)
TRANSITION = np.array([[ 0.+0.j,  -1.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                       [ 1.+0.j,   0.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,   0.+0j, 1.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,   0.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,   0.+0j, 0.+0.j, 0.+0.j, 0.+0.j ]], dtype=np.csingle)
SM = VECT * np.conjugate(VECT).transpose()
REF = np.matmul(np.matmul(TRANSITION.transpose(), SM), TRANSITION)
np.all(REF == lfr.Matrix_change_of_basis(SM,TRANSITION))

TRANSITION = np.array([[ 1.+0.j,  0.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,  1.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,  0.+0j, 1.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,  0.+0j, 0.+0.j, 1.+0.j, 0.+0.j ],
                       [ 0.+0.j,  0.+0j, 0.+0.j, 0.+0.j, 1.+0.j ]], dtype=np.csingle)
