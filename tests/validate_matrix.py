import numpy

def check_numpyarray(A, B):
    m,n = A.shape
    for i in range(m):
        for j in range(n):
            try:
                assert numpy.isclose(A[i,j],B[i,j])
            except AssertionError:
                raise ValueError(f'The values are not same for row: {i+1} and column: {j+1}, as {A[i,j]} is not equal/close to {B[i,j]}!')
                