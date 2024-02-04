from src.acrod import urdf_to_robottopologymatrix
import os
import numpy
from tests.validate_matrix import check_numpyarray

def test_urdf_to_robottopologymatrix():
    M1 = urdf_to_robottopologymatrix.from_urdf_to_matrix(os.path.join(os.getcwd(),r'misc',r'urdf','puma560_robot.urdf'))
    M2 = urdf_to_robottopologymatrix.from_urdf_to_matrix(os.path.join(os.getcwd(),r'misc',r'urdf','scara_52900.urdf.xacro'))

    #assert numpy.isclose(M1,numpy.array([[9, 1, 0, 0, 0, 0, 0], [1, 9, 1, 0, 0, 0, 0], [0, 1, 9, 1, 0, 0, 0], [0, 0, 1, 9, 1, 0, 0], [0, 0, 0, 1, 9, 1, 0], [0, 0, 0, 0, 1, 9, 1], [0, 0, 0, 0, 0, 1, 9]])).all()
    #assert numpy.isclose(M2,numpy.array([[9, 0, 0, 1, 0], [0, 9, 1, 0, 1], [0, 1, 9, 2, 0], [1, 0, 1, 9, 0], [0, 1, 0, 0, 9]])).all()

    if not numpy.isclose(M1,numpy.array([[9, 1, 0, 0, 0, 0, 0], [1, 9, 1, 0, 0, 0, 0], [0, 1, 9, 1, 0, 0, 0], [0, 0, 1, 9, 1, 0, 0], [0, 0, 0, 1, 9, 1, 0], [0, 0, 0, 0, 1, 9, 1], [0, 0, 0, 0, 0, 1, 9]])).all():
        check_numpyarray(M1,numpy.array([[9, 1, 0, 0, 0, 0, 0], [1, 9, 1, 0, 0, 0, 0], [0, 1, 9, 1, 0, 0, 0], [0, 0, 1, 9, 1, 0, 0], [0, 0, 0, 1, 9, 1, 0], [0, 0, 0, 0, 1, 9, 1], [0, 0, 0, 0, 0, 1, 9]]))

    if not numpy.isclose(M2,numpy.array([[9, 0, 0, 1, 0], [0, 9, 1, 0, 1], [0, 1, 9, 2, 0], [1, 0, 1, 9, 0], [0, 1, 0, 0, 9]])).all():
        check_numpyarray(M2,numpy.array([[9, 0, 0, 1, 0], [0, 9, 1, 0, 1], [0, 1, 9, 2, 0], [1, 0, 1, 9, 0], [0, 1, 0, 0, 9]]))
