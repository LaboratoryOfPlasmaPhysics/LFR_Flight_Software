#!/usr/bin/env python

import unittest
import lfr
import numpy as np
from ddt import ddt, data, unpack

def cplx_random(shape, dtype=np.csingle):
    return (np.random.random(shape) + np.random.random(shape) * 1j).astype(dtype)

def make_SM(Vect):
    return Vect.transpose() @ np.conjugate(Vect)

def make_transition_matrix(B_cal_matrix, E_cal_matrix):
    transition = np.zeros((5,5), dtype=np.csingle)
    transition[:3,:3]=B_cal_matrix
    transition[3:,3:]=E_cal_matrix
    return transition

def extract_triangular_matrix_ref(M):
    tri = []
    for line in range(5):
        for column in range(5):
            if line <= column:
                tri.append(M[line, column].real)
                if line < column:
                    tri.append(M[line, column].imag)
    return tri


def convert_ASMs_to_VHDL_repr_ref(ASM):
    vhdl_repr = [None]*25*128
    index = 0
    for line in range(5):
        for column in range(5):
            if line == column:
                vhdl_repr[index:index+128] = ASM[:, line, column].real
                index += 128
            elif line < column:
                vhdl_repr[index:index+256:2] = ASM[:, line, column].real
                vhdl_repr[index+1:index+256:2] = ASM[:, line, column].imag
                index += 256
    return vhdl_repr



@ddt
class PythonWrappers(unittest.TestCase):

    @data(*[(make_SM(cplx_random((1,5))), ) for i in range(100)])
    @unpack
    def test_can_extract_an_lfr_sm_from_a_full_matrix(self, SM, *args):
        self.assertTrue(lfr.Extract_triangular_matrix(SM) == extract_triangular_matrix_ref(SM))

    def test_can_extract_an_lfr_sm_from_a_full_matrix(self):
        ASM=np.zeros((128,5,5), dtype=np.csingle)
        for f in range(128):
            ASM[f] = make_SM(cplx_random((1,5))*f)
        self.assertTrue(lfr.Convert_ASMs_to_VHDL_repr(ASM) == convert_ASMs_to_VHDL_repr_ref(ASM))

@ddt
class AnLFRMatrix(unittest.TestCase):
   @data(*[(i*cplx_random((1,5)), i*cplx_random((3,3)) , i*cplx_random((2,2))) for i in range(100)])
   @unpack
   def test_can_be_calibrated(self,VECT, BCAL, ECAL):
       transition = make_transition_matrix(BCAL, ECAL)
       SM = make_SM(VECT)
       REF = transition @ SM @ transition.transpose().conjugate()
       self.assertTrue(np.allclose(lfr.Matrix_change_of_basis(SM, transition), REF, rtol=1e-06, atol=0.))

class AnLFRSpectralMatrix(unittest.TestCase):
  def test_can_be_calibrated(self):
      ASM=np.zeros((128,5,5), dtype=np.csingle)
      CAL_MATRICES=np.zeros((128,5,5), dtype=np.csingle)
      CALIBRATED_ASM=np.zeros((128,5,5), dtype=np.csingle)
      for f in range(128):
          transition = make_transition_matrix(cplx_random((3,3))*f, cplx_random((2,2))*f)
          SM = make_SM(cplx_random((1,5))*f)
          REF = transition @ SM @ transition.transpose().conjugate()
          ASM[f] = SM
          CAL_MATRICES[f] = transition
          CALIBRATED_ASM[f] = REF
      self.assertTrue(np.allclose(lfr.SM_calibrate_and_reorder(ASM,CAL_MATRICES), CALIBRATED_ASM, rtol=1e-06, atol=0.))

if __name__ == '__main__':
   unittest.main()
