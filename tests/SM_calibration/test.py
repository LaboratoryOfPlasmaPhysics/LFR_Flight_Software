#!/usr/bin/env python

import unittest
import lfr
import numpy as np



TRANSITION = np.array([[ 1.+0.j,  0.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,  1.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,  0.+0j, 1.+0.j, 0.+0.j, 0.+0.j ],
                       [ 0.+0.j,  0.+0j, 0.+0.j, 1.+0.j, 0.+0.j ],
                       [ 0.+0.j,  0.+0j, 0.+0.j, 0.+0.j, 1.+0.j ]], dtype=np.csingle)

class AnLFRMatrix(unittest.TestCase):
   def test_can_be_calibrated(self):
       B1=1.+0.j
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
       self.assertTrue(np.all(REF == lfr.Matrix_change_of_basis(SM,TRANSITION)))

class AnLFRSpectralMatrix(unittest.TestCase):
  def test_can_be_calibrated(self):
      ASM=np.zeros((128,5,5), dtype=np.csingle)
      CAL_MATRICES=np.zeros((128,5,5), dtype=np.csingle)
      CALIBRATED_ASM=np.zeros((128,5,5), dtype=np.csingle)
      for f in range(128):
          B1=f*1.+0.j
          B2=0.+1.j
          B3=1.+1.j
          E1=3.+1.j
          E2=1.+3.j
          VECT = np.array([[B1,B2,B3,E1,E2]], dtype=np.csingle)
          TRANSITION = np.array([[ 0.+0.j,  -1.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                                 [ 1.+0.j,   0.+0j, 0.+0.j, 0.+0.j, 0.+0.j ],
                                 [ 0.+0.j,   0.+0j, 1.+0.j, 0.+0.j, 0.+0.j ],
                                 [ 0.+0.j,   0.+0j, 0.+0.j, f +0.j, 0.+0.j ],
                                 [ 0.+0.j,   0.+0j, 0.+0.j, 0.+0.j, f +0.j ]], dtype=np.csingle)
          SM = VECT * np.conjugate(VECT).transpose()
          REF = np.matmul(np.matmul(TRANSITION.transpose(), SM), TRANSITION)
          ASM[f] = SM
          CAL_MATRICES[f] = TRANSITION
          CALIBRATED_ASM[f] = REF
      self.assertTrue(np.all(CALIBRATED_ASM == lfr.SM_calibrate(ASM,CAL_MATRICES)))

if __name__ == '__main__':
   unittest.main()
