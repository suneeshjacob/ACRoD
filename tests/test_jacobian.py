from src.jacobian_spatial import jacobian as jacobian_spatial, finalconnections
from src.jacobian_planar import jacobian as jacobian_planar
from new_version.functions import all_paths

import numpy

def test_planar_3Rs():
    A = numpy.matrix('9 1 0 0;1 9 1 0;0 1 9 1;0 0 1 9')
    assert str(jacobian_planar(A)[0]) == 'Matrix([[-a_y + r_{(1,2)y}, -a_y + r_{(2,3)y}, -a_y + r_{(3,4)y}], [a_x - r_{(1,2)x}, a_x - r_{(2,3)x}, a_x - r_{(3,4)x}], [1, 1, 1]])'

def test_planar_3RPRp():
    A = numpy.matrix('9 1 0 1 0 1 0 0;0 9 2 0 0 0 0 0;0 1 9 0 0 0 0 1;0 0 0 9 2 0 0 0;0 0 0 1 9 0 0 1;0 0 0 0 0 9 2 0;0 0 0 0 0 1 9 1;0 0 0 0 0 0 0 9')
    assert str(jacobian_planar(A)[:3]) == '[Matrix([\n[n_{(2,3)x}, 0, 0],\n[n_{(2,3)y}, 0, 0],\n[         0, 0, 0]]), Matrix([\n[-a_y + r_{(1,2)y}, 0, 0, -a_y + r_{(3,8)y}, 0, 0],\n[ a_x - r_{(1,2)x}, 0, 0,  a_x - r_{(3,8)x}, 0, 0],\n[                1, 0, 0,                 1, 0, 0]]), Matrix([\n[          0,          0,          0],\n[-n_{(2,3)x},          0, n_{(6,7)x}],\n[-n_{(2,3)y},          0, n_{(6,7)y}],\n[          0,          0,          0],\n[-n_{(2,3)x}, n_{(4,5)x},          0],\n[-n_{(2,3)y}, n_{(4,5)y},          0]])]'
    
def test_planar_5Rp():
    A = numpy.matrix('9 1 0 1 0;1 9 1 0 0;0 0 9 0 1;1 0 0 9 1;0 0 0 0 9')
    assert str(jacobian_planar(A)[:3]) == '[Matrix([\n[-a_y + r_{(1,2)y}, 0],\n[ a_x - r_{(1,2)x}, 0],\n[                1, 0]]), Matrix([\n[-a_y + r_{(2,3)y}, -a_y + r_{(3,5)y}, 0],\n[ a_x - r_{(2,3)x},  a_x - r_{(3,5)x}, 0],\n[                1,                 1, 0]]), Matrix([\n[               -1,                 1],\n[ a_y - r_{(1,2)y}, -a_y + r_{(1,4)y}],\n[-a_x + r_{(1,2)x},  a_x - r_{(1,4)x}]])]'

def test_planar_4R4Psp():
    A = numpy.matrix('9 2 2 0 0 0 0;1 9 0 2 0 0 0;1 0 9 2 0 0 0;0 0 0 9 1 1 0;0 0 0 1 9 0 1;0 0 0 0 0 9 1;0 0 0 0 0 0 9')
    assert str(jacobian_planar(A)[:3]) == '[Matrix([\n[-a_y + r_{(4,5)y}, n_{(1,2)x}, 0],\n[ a_x - r_{(4,5)x}, n_{(1,2)y}, 0],\n[                1,          0, 0]]), Matrix([\n[0, -a_y + r_{(5,7)y}, 0, n_{(2,4)x}, 0],\n[0,  a_x - r_{(5,7)x}, 0, n_{(2,4)y}, 0],\n[0,                 1, 0,          0, 0]]), Matrix([\n[               -1,           0,          0],\n[ a_y - r_{(4,5)y},           0,          0],\n[-a_x + r_{(4,5)x},           0,          0],\n[                0, -n_{(1,2)x}, n_{(1,3)x}],\n[                0, -n_{(1,2)y}, n_{(1,3)y}]])]'

def test_spatial_6Rs():
    A = numpy.matrix('9 1 0 0 0 0 0;1 9 1 0 0 0 0;0 1 9 1 0 0 0;0 0 1 9 1 0 0;0 0 0 1 9 1 0;0 0 0 0 1 9 1;0 0 0 0 0 1 9')
    assert str(jacobian_spatial(A)[0]) == 'Matrix([[n_{(1,2)y}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_y - r_{(1,2)y}), n_{(2,3)y}*(a_z - r_{(2,3)z}) - n_{(2,3)z}*(a_y - r_{(2,3)y}), n_{(3,4)y}*(a_z - r_{(3,4)z}) - n_{(3,4)z}*(a_y - r_{(3,4)y}), n_{(4,5)y}*(a_z - r_{(4,5)z}) - n_{(4,5)z}*(a_y - r_{(4,5)y}), n_{(5,6)y}*(a_z - r_{(5,6)z}) - n_{(5,6)z}*(a_y - r_{(5,6)y}), n_{(6,7)y}*(a_z - r_{(6,7)z}) - n_{(6,7)z}*(a_y - r_{(6,7)y})], [-n_{(1,2)x}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_x - r_{(1,2)x}), -n_{(2,3)x}*(a_z - r_{(2,3)z}) + n_{(2,3)z}*(a_x - r_{(2,3)x}), -n_{(3,4)x}*(a_z - r_{(3,4)z}) + n_{(3,4)z}*(a_x - r_{(3,4)x}), -n_{(4,5)x}*(a_z - r_{(4,5)z}) + n_{(4,5)z}*(a_x - r_{(4,5)x}), -n_{(5,6)x}*(a_z - r_{(5,6)z}) + n_{(5,6)z}*(a_x - r_{(5,6)x}), -n_{(6,7)x}*(a_z - r_{(6,7)z}) + n_{(6,7)z}*(a_x - r_{(6,7)x})], [n_{(1,2)x}*(a_y - r_{(1,2)y}) - n_{(1,2)y}*(a_x - r_{(1,2)x}), n_{(2,3)x}*(a_y - r_{(2,3)y}) - n_{(2,3)y}*(a_x - r_{(2,3)x}), n_{(3,4)x}*(a_y - r_{(3,4)y}) - n_{(3,4)y}*(a_x - r_{(3,4)x}), n_{(4,5)x}*(a_y - r_{(4,5)y}) - n_{(4,5)y}*(a_x - r_{(4,5)x}), n_{(5,6)x}*(a_y - r_{(5,6)y}) - n_{(5,6)y}*(a_x - r_{(5,6)x}), n_{(6,7)x}*(a_y - r_{(6,7)y}) - n_{(6,7)y}*(a_x - r_{(6,7)x})], [n_{(1,2)x}, n_{(2,3)x}, n_{(3,4)x}, n_{(4,5)x}, n_{(5,6)x}, n_{(6,7)x}], [n_{(1,2)y}, n_{(2,3)y}, n_{(3,4)y}, n_{(4,5)y}, n_{(5,6)y}, n_{(6,7)y}], [n_{(1,2)z}, n_{(2,3)z}, n_{(3,4)z}, n_{(4,5)z}, n_{(5,6)z}, n_{(6,7)z}]])'

def test_spatial_RSSRp():
    A = numpy.matrix('9 1 0 1;1 9 4 0;0 0 9 4;0 0 0 9')
    assert str(jacobian_spatial(A)[:3]) == '[Matrix([\n[ n_{(1,2)y}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_y - r_{(1,2)y})],\n[-n_{(1,2)x}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_x - r_{(1,2)x})],\n[ n_{(1,2)x}*(a_y - r_{(1,2)y}) - n_{(1,2)y}*(a_x - r_{(1,2)x})],\n[                                                    n_{(1,2)x}],\n[                                                    n_{(1,2)y}],\n[                                                    n_{(1,2)z}]]), Matrix([\n[0,                 0,  a_z - r_{(2,3)z}, -a_y + r_{(2,3)y},                 0,  a_z - r_{(3,4)z}, -a_y + r_{(3,4)y}],\n[0, -a_z + r_{(2,3)z},                 0,  a_x - r_{(2,3)x}, -a_z + r_{(3,4)z},                 0,  a_x - r_{(3,4)x}],\n[0,  a_y - r_{(2,3)y}, -a_x + r_{(2,3)x},                 0,  a_y - r_{(3,4)y}, -a_x + r_{(3,4)x},                 0],\n[0,                 1,                 0,                 0,                 1,                 0,                 0],\n[0,                 0,                 1,                 0,                 0,                 1,                 0],\n[0,                 0,                 0,                 1,                 0,                 0,                 1]]), Matrix([\n[                                                                                                          -n_{(1,2)x}],\n[                                                                                                          -n_{(1,2)y}],\n[                                                                                                          -n_{(1,2)z}],\n[                                                       -n_{(1,2)y}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_y - r_{(1,2)y})],\n[                                                        n_{(1,2)x}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_x - r_{(1,2)x})],\n[                                                       -n_{(1,2)x}*(a_y - r_{(1,2)y}) + n_{(1,2)y}*(a_x - r_{(1,2)x})],\n[n_{(1,2)x}*(-r_{(2,3)x} + r_{(3,4)x}) + n_{(1,2)y}*(-r_{(2,3)y} + r_{(3,4)y}) + n_{(1,2)z}*(-r_{(2,3)z} + r_{(3,4)z})]])]'

def test_spatial_RSSRSSR():
    A = numpy.matrix('9 1 1 0 1 0;1 9 0 0 0 4;1 0 9 4 0 0;0 0 0 9 0 4;0 0 0 0 9 4;0 0 0 0 0 9')
    assert str(jacobian_spatial(A)[:3]) == '[Matrix([\n[0,  n_{(1,3)y}*(a_z - r_{(1,3)z}) - n_{(1,3)z}*(a_y - r_{(1,3)y})],\n[0, -n_{(1,3)x}*(a_z - r_{(1,3)z}) + n_{(1,3)z}*(a_x - r_{(1,3)x})],\n[0,  n_{(1,3)x}*(a_y - r_{(1,3)y}) - n_{(1,3)y}*(a_x - r_{(1,3)x})],\n[0,                                                     n_{(1,3)x}],\n[0,                                                     n_{(1,3)y}],\n[0,                                                     n_{(1,3)z}]]), Matrix([\n[0, 0, 0, 0,                 0,  a_z - r_{(3,4)z}, -a_y + r_{(3,4)y},                 0,  a_z - r_{(4,6)z}, -a_y + r_{(4,6)y}, 0, 0, 0],\n[0, 0, 0, 0, -a_z + r_{(3,4)z},                 0,  a_x - r_{(3,4)x}, -a_z + r_{(4,6)z},                 0,  a_x - r_{(4,6)x}, 0, 0, 0],\n[0, 0, 0, 0,  a_y - r_{(3,4)y}, -a_x + r_{(3,4)x},                 0,  a_y - r_{(4,6)y}, -a_x + r_{(4,6)x},                 0, 0, 0, 0],\n[0, 0, 0, 0,                 1,                 0,                 0,                 1,                 0,                 0, 0, 0, 0],\n[0, 0, 0, 0,                 0,                 1,                 0,                 0,                 1,                 0, 0, 0, 0],\n[0, 0, 0, 0,                 0,                 0,                 1,                 0,                 0,                 1, 0, 0, 0]]), Matrix([\n[                                                             0,                                                                                                           -n_{(1,3)x}],\n[                                                             0,                                                                                                           -n_{(1,3)y}],\n[                                                             0,                                                                                                           -n_{(1,3)z}],\n[                                                             0,                                                        -n_{(1,3)y}*(a_z - r_{(1,3)z}) + n_{(1,3)z}*(a_y - r_{(1,3)y})],\n[                                                             0,                                                         n_{(1,3)x}*(a_z - r_{(1,3)z}) - n_{(1,3)z}*(a_x - r_{(1,3)x})],\n[                                                             0,                                                        -n_{(1,3)x}*(a_y - r_{(1,3)y}) + n_{(1,3)y}*(a_x - r_{(1,3)x})],\n[                                                    n_{(1,2)x},                                                                                                           -n_{(1,3)x}],\n[                                                    n_{(1,2)y},                                                                                                           -n_{(1,3)y}],\n[                                                    n_{(1,2)z},                                                                                                           -n_{(1,3)z}],\n[ n_{(1,2)y}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_y - r_{(1,2)y}),                                                        -n_{(1,3)y}*(a_z - r_{(1,3)z}) + n_{(1,3)z}*(a_y - r_{(1,3)y})],\n[-n_{(1,2)x}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_x - r_{(1,2)x}),                                                         n_{(1,3)x}*(a_z - r_{(1,3)z}) - n_{(1,3)z}*(a_x - r_{(1,3)x})],\n[ n_{(1,2)x}*(a_y - r_{(1,2)y}) - n_{(1,2)y}*(a_x - r_{(1,2)x}),                                                        -n_{(1,3)x}*(a_y - r_{(1,3)y}) + n_{(1,3)y}*(a_x - r_{(1,3)x})],\n[                                                             0, n_{(1,3)x}*(-r_{(3,4)x} + r_{(4,6)x}) + n_{(1,3)y}*(-r_{(3,4)y} + r_{(4,6)y}) + n_{(1,3)z}*(-r_{(3,4)z} + r_{(4,6)z})]])]'

def test_6UPS():
    A = numpy.matrix('9 5 0 5 0 5 0 5 0 5 0 5 0 0;0 9 2 0 0 0 0 0 0 0 0 0 0 0;0 1 9 0 0 0 0 0 0 0 0 0 0 4;0 0 0 9 2 0 0 0 0 0 0 0 0 0;0 0 0 1 9 0 0 0 0 0 0 0 0 4;0 0 0 0 0 9 2 0 0 0 0 0 0 0;0 0 0 0 0 1 9 0 0 0 0 0 0 4;0 0 0 0 0 0 0 9 2 0 0 0 0 0;0 0 0 0 0 0 0 1 9 0 0 0 0 4;0 0 0 0 0 0 0 0 0 9 2 0 0 0;0 0 0 0 0 0 0 0 0 1 9 0 0 4;0 0 0 0 0 0 0 0 0 0 0 9 2 0;0 0 0 0 0 0 0 0 0 0 0 1 9 4;0 0 0 0 0 0 0 0 0 0 0 0 0 9')
    assert str(jacobian_spatial(A)[:3]) == '[Matrix([\n[0, 0, n_{(2,3)x}, 0, 0, 0],\n[0, 0, n_{(2,3)y}, 0, 0, 0],\n[0, 0, n_{(2,3)z}, 0, 0, 0],\n[0, 0,          0, 0, 0, 0],\n[0, 0,          0, 0, 0, 0],\n[0, 0,          0, 0, 0, 0]]), Matrix([\n[0, 0,  m_{(1,2)y}*(a_z - r_{(1,2)z}) - m_{(1,2)z}*(a_y - r_{(1,2)y}), 0, 0, 0, 0, 0,  n_{(1,2)y}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_y - r_{(1,2)y}), 0, 0, 0, 0, 0, 0, 0, 0, 0,                  0,  a_z - r_{(3,14)z}, -a_y + r_{(3,14)y}, 0, 0, 0, 0, 0, 0, 0, 0, 0],\n[0, 0, -m_{(1,2)x}*(a_z - r_{(1,2)z}) + m_{(1,2)z}*(a_x - r_{(1,2)x}), 0, 0, 0, 0, 0, -n_{(1,2)x}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_x - r_{(1,2)x}), 0, 0, 0, 0, 0, 0, 0, 0, 0, -a_z + r_{(3,14)z},                  0,  a_x - r_{(3,14)x}, 0, 0, 0, 0, 0, 0, 0, 0, 0],\n[0, 0,  m_{(1,2)x}*(a_y - r_{(1,2)y}) - m_{(1,2)y}*(a_x - r_{(1,2)x}), 0, 0, 0, 0, 0,  n_{(1,2)x}*(a_y - r_{(1,2)y}) - n_{(1,2)y}*(a_x - r_{(1,2)x}), 0, 0, 0, 0, 0, 0, 0, 0, 0,  a_y - r_{(3,14)y}, -a_x + r_{(3,14)x},                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\n[0, 0,                                                     m_{(1,2)x}, 0, 0, 0, 0, 0,                                                     n_{(1,2)x}, 0, 0, 0, 0, 0, 0, 0, 0, 0,                  1,                  0,                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\n[0, 0,                                                     m_{(1,2)y}, 0, 0, 0, 0, 0,                                                     n_{(1,2)y}, 0, 0, 0, 0, 0, 0, 0, 0, 0,                  0,                  1,                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\n[0, 0,                                                     m_{(1,2)z}, 0, 0, 0, 0, 0,                                                     n_{(1,2)z}, 0, 0, 0, 0, 0, 0, 0, 0, 0,                  0,                  0,                  1, 0, 0, 0, 0, 0, 0, 0, 0, 0]]), Matrix([\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0, n_{(12,13)x}, -n_{(2,3)x},          0,          0,          0],\n[           0, n_{(12,13)y}, -n_{(2,3)y},          0,          0,          0],\n[           0, n_{(12,13)z}, -n_{(2,3)z},          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[n_{(10,11)x},            0, -n_{(2,3)x},          0,          0,          0],\n[n_{(10,11)y},            0, -n_{(2,3)y},          0,          0,          0],\n[n_{(10,11)z},            0, -n_{(2,3)z},          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0, -n_{(2,3)x},          0,          0, n_{(8,9)x}],\n[           0,            0, -n_{(2,3)y},          0,          0, n_{(8,9)y}],\n[           0,            0, -n_{(2,3)z},          0,          0, n_{(8,9)z}],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0, -n_{(2,3)x},          0, n_{(6,7)x},          0],\n[           0,            0, -n_{(2,3)y},          0, n_{(6,7)y},          0],\n[           0,            0, -n_{(2,3)z},          0, n_{(6,7)z},          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0,           0,          0,          0,          0],\n[           0,            0, -n_{(2,3)x}, n_{(4,5)x},          0,          0],\n[           0,            0, -n_{(2,3)y}, n_{(4,5)y},          0,          0],\n[           0,            0, -n_{(2,3)z}, n_{(4,5)z},          0,          0]])]'
    
def test_SPM():
    A = numpy.matrix('9,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0;0,9,2,0,0,0,0,0,0,0,0,0,0,0,0,0;0,1,9,0,0,0,0,4,0,0,0,0,0,0,0,0;0,0,0,9,2,0,0,0,0,0,0,0,0,0,0,0;0,0,0,1,9,0,0,4,0,0,0,0,0,0,0,0;0,0,0,0,0,9,2,0,0,0,0,0,0,0,0,0;0,0,0,0,0,1,9,4,0,0,0,0,0,0,0,0;0,0,0,0,0,0,0,9,4,0,4,0,4,0,5,0;0,0,0,0,0,0,0,0,9,2,0,0,0,0,0,0;0,0,0,0,0,0,0,0,1,9,0,0,0,0,0,4;0,0,0,0,0,0,0,0,0,0,9,2,0,0,0,0;0,0,0,0,0,0,0,0,0,0,1,9,0,0,0,4;0,0,0,0,0,0,0,0,0,0,0,0,9,2,0,0;0,0,0,0,0,0,0,0,0,0,0,0,1,9,0,4;0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,2;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9')
    assert str(jacobian_spatial(A)[:3]) == '[Matrix([\n[0, 0, n_{(2,3)x}, 0, 0, n_{(9,10)x}],\n[0, 0, n_{(2,3)y}, 0, 0, n_{(9,10)y}],\n[0, 0, n_{(2,3)z}, 0, 0, n_{(9,10)z}],\n[0, 0,          0, 0, 0,           0],\n[0, 0,          0, 0, 0,           0],\n[0, 0,          0, 0, 0,           0]]), Matrix([\n[0,  n_{(1,2)y}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_y - r_{(1,2)y}), 0, 0, 0, 0,                   0,  a_z - r_{(10,16)z}, -a_y + r_{(10,16)y}, 0, 0, 0, 0, 0, 0,                 0,  a_z - r_{(3,8)z}, -a_y + r_{(3,8)y}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                 0,  a_z - r_{(8,9)z}, -a_y + r_{(8,9)y}],\n[0, -n_{(1,2)x}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_x - r_{(1,2)x}), 0, 0, 0, 0, -a_z + r_{(10,16)z},                   0,  a_x - r_{(10,16)x}, 0, 0, 0, 0, 0, 0, -a_z + r_{(3,8)z},                 0,  a_x - r_{(3,8)x}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -a_z + r_{(8,9)z},                 0,  a_x - r_{(8,9)x}],\n[0,  n_{(1,2)x}*(a_y - r_{(1,2)y}) - n_{(1,2)y}*(a_x - r_{(1,2)x}), 0, 0, 0, 0,  a_y - r_{(10,16)y}, -a_x + r_{(10,16)x},                   0, 0, 0, 0, 0, 0, 0,  a_y - r_{(3,8)y}, -a_x + r_{(3,8)x},                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  a_y - r_{(8,9)y}, -a_x + r_{(8,9)x},                 0],\n[0,                                                     n_{(1,2)x}, 0, 0, 0, 0,                   1,                   0,                   0, 0, 0, 0, 0, 0, 0,                 1,                 0,                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                 1,                 0,                 0],\n[0,                                                     n_{(1,2)y}, 0, 0, 0, 0,                   0,                   1,                   0, 0, 0, 0, 0, 0, 0,                 0,                 1,                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                 0,                 1,                 0],\n[0,                                                     n_{(1,2)z}, 0, 0, 0, 0,                   0,                   0,                   1, 0, 0, 0, 0, 0, 0,                 0,                 0,                 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                 0,                 0,                 1]]), Matrix([\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0, -n_{(9,10)x}],\n[           0,            0,           0,          0,          0, -n_{(9,10)y}],\n[           0,            0,           0,          0,          0, -n_{(9,10)z}],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0, n_{(13,14)x},           0,          0,          0, -n_{(9,10)x}],\n[           0, n_{(13,14)y},           0,          0,          0, -n_{(9,10)y}],\n[           0, n_{(13,14)z},           0,          0,          0, -n_{(9,10)z}],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[n_{(11,12)x},            0,           0,          0,          0, -n_{(9,10)x}],\n[n_{(11,12)y},            0,           0,          0,          0, -n_{(9,10)y}],\n[n_{(11,12)z},            0,           0,          0,          0, -n_{(9,10)z}],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0, -n_{(2,3)x},          0, n_{(6,7)x},            0],\n[           0,            0, -n_{(2,3)y},          0, n_{(6,7)y},            0],\n[           0,            0, -n_{(2,3)z},          0, n_{(6,7)z},            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0, -n_{(2,3)x}, n_{(4,5)x},          0,            0],\n[           0,            0, -n_{(2,3)y}, n_{(4,5)y},          0,            0],\n[           0,            0, -n_{(2,3)z}, n_{(4,5)z},          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0],\n[           0,            0,           0,          0,          0,            0]])]'
    
def test_6CPU():
    A = numpy.matrix('9 3 0 3 0 3 0 0;0 9 2 0 0 0 0 0;0 1 9 0 0 0 0 5;0 0 0 9 2 0 0 0;0 0 0 1 9 0 0 5;0 0 0 0 0 9 2 0;0 0 0 0 0 1 9 5;0 0 0 0 0 0 0 9')
    assert str(jacobian_spatial(A)[:3]) == '[Matrix([\n[n_{(2,3)x}, 0, 0],\n[n_{(2,3)y}, 0, 0],\n[n_{(2,3)z}, 0, 0],\n[         0, 0, 0],\n[         0, 0, 0],\n[         0, 0, 0]]), Matrix([\n[ m_{(3,8)y}*(a_z - r_{(3,8)z}) - m_{(3,8)z}*(a_y - r_{(3,8)y}), 0, 0,  n_{(1,2)y}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_y - r_{(1,2)y}), 0, 0,  n_{(3,8)y}*(a_z - r_{(3,8)z}) - n_{(3,8)z}*(a_y - r_{(3,8)y}), 0, 0, n_{(1,2)x}, 0, 0],\n[-m_{(3,8)x}*(a_z - r_{(3,8)z}) + m_{(3,8)z}*(a_x - r_{(3,8)x}), 0, 0, -n_{(1,2)x}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_x - r_{(1,2)x}), 0, 0, -n_{(3,8)x}*(a_z - r_{(3,8)z}) + n_{(3,8)z}*(a_x - r_{(3,8)x}), 0, 0, n_{(1,2)y}, 0, 0],\n[ m_{(3,8)x}*(a_y - r_{(3,8)y}) - m_{(3,8)y}*(a_x - r_{(3,8)x}), 0, 0,  n_{(1,2)x}*(a_y - r_{(1,2)y}) - n_{(1,2)y}*(a_x - r_{(1,2)x}), 0, 0,  n_{(3,8)x}*(a_y - r_{(3,8)y}) - n_{(3,8)y}*(a_x - r_{(3,8)x}), 0, 0, n_{(1,2)z}, 0, 0],\n[                                                    m_{(3,8)x}, 0, 0,                                                     n_{(1,2)x}, 0, 0,                                                     n_{(3,8)x}, 0, 0,          0, 0, 0],\n[                                                    m_{(3,8)y}, 0, 0,                                                     n_{(1,2)y}, 0, 0,                                                     n_{(3,8)y}, 0, 0,          0, 0, 0],\n[                                                    m_{(3,8)z}, 0, 0,                                                     n_{(1,2)z}, 0, 0,                                                     n_{(3,8)z}, 0, 0,          0, 0, 0]]), Matrix([\n[          0,          0,          0],\n[          0,          0,          0],\n[          0,          0,          0],\n[-n_{(2,3)x},          0, n_{(6,7)x}],\n[-n_{(2,3)y},          0, n_{(6,7)y}],\n[-n_{(2,3)z},          0, n_{(6,7)z}],\n[          0,          0,          0],\n[          0,          0,          0],\n[          0,          0,          0],\n[-n_{(2,3)x}, n_{(4,5)x},          0],\n[-n_{(2,3)y}, n_{(4,5)y},          0],\n[-n_{(2,3)z}, n_{(4,5)z},          0]])]'

def test_R2SR():
    A = numpy.matrix('9 1 0 1;1 9 4 0;0 0 9 4;0 0 0 9')
    assert str(jacobian_spatial(A)[:3]) == '[Matrix([\n[ n_{(1,2)y}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_y - r_{(1,2)y})],\n[-n_{(1,2)x}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_x - r_{(1,2)x})],\n[ n_{(1,2)x}*(a_y - r_{(1,2)y}) - n_{(1,2)y}*(a_x - r_{(1,2)x})],\n[                                                    n_{(1,2)x}],\n[                                                    n_{(1,2)y}],\n[                                                    n_{(1,2)z}]]), Matrix([\n[0,                 0,  a_z - r_{(2,3)z}, -a_y + r_{(2,3)y},                 0,  a_z - r_{(3,4)z}, -a_y + r_{(3,4)y}],\n[0, -a_z + r_{(2,3)z},                 0,  a_x - r_{(2,3)x}, -a_z + r_{(3,4)z},                 0,  a_x - r_{(3,4)x}],\n[0,  a_y - r_{(2,3)y}, -a_x + r_{(2,3)x},                 0,  a_y - r_{(3,4)y}, -a_x + r_{(3,4)x},                 0],\n[0,                 1,                 0,                 0,                 1,                 0,                 0],\n[0,                 0,                 1,                 0,                 0,                 1,                 0],\n[0,                 0,                 0,                 1,                 0,                 0,                 1]]), Matrix([\n[                                                                                                          -n_{(1,2)x}],\n[                                                                                                          -n_{(1,2)y}],\n[                                                                                                          -n_{(1,2)z}],\n[                                                       -n_{(1,2)y}*(a_z - r_{(1,2)z}) + n_{(1,2)z}*(a_y - r_{(1,2)y})],\n[                                                        n_{(1,2)x}*(a_z - r_{(1,2)z}) - n_{(1,2)z}*(a_x - r_{(1,2)x})],\n[                                                       -n_{(1,2)x}*(a_y - r_{(1,2)y}) + n_{(1,2)y}*(a_x - r_{(1,2)x})],\n[n_{(1,2)x}*(-r_{(2,3)x} + r_{(3,4)x}) + n_{(1,2)y}*(-r_{(2,3)y} + r_{(3,4)y}) + n_{(1,2)z}*(-r_{(2,3)z} + r_{(3,4)z})]])]'

def test_allpathsfunction():
    A_list = [numpy.matrix('9 1 0 0;1 9 1 0;0 1 9 1;0 0 1 9'),numpy.matrix('9 1 0 1 0 1 0 0;0 9 2 0 0 0 0 0;0 1 9 0 0 0 0 1;0 0 0 9 2 0 0 0;0 0 0 1 9 0 0 1;0 0 0 0 0 9 2 0;0 0 0 0 0 1 9 1;0 0 0 0 0 0 0 9'),numpy.matrix('9 1 0 1 0;1 9 1 0 0;0 0 9 0 1;1 0 0 9 1;0 0 0 0 9'),numpy.matrix('9 1 0 0 0 0 0;1 9 1 0 0 0 0;0 1 9 1 0 0 0;0 0 1 9 1 0 0;0 0 0 1 9 1 0;0 0 0 0 1 9 1;0 0 0 0 0 1 9'),numpy.matrix('9 1 0 1;1 9 4 0;0 0 9 4;0 0 0 9'),numpy.matrix('9 2 2 0 0 0 0;1 9 0 2 0 0 0;1 0 9 2 0 0 0;0 0 0 9 1 1 0;0 0 0 1 9 0 1;0 0 0 0 0 9 1;0 0 0 0 0 0 9'),numpy.matrix('9 1 1 0 1 0;1 9 0 0 0 4;1 0 9 4 0 0;0 0 0 9 0 4;0 0 0 0 9 4;0 0 0 0 0 9'),numpy.matrix('9 5 0 5 0 5 0 5 0 5 0 5 0 0;0 9 2 0 0 0 0 0 0 0 0 0 0 0;0 1 9 0 0 0 0 0 0 0 0 0 0 4;0 0 0 9 2 0 0 0 0 0 0 0 0 0;0 0 0 1 9 0 0 0 0 0 0 0 0 4;0 0 0 0 0 9 2 0 0 0 0 0 0 0;0 0 0 0 0 1 9 0 0 0 0 0 0 4;0 0 0 0 0 0 0 9 2 0 0 0 0 0;0 0 0 0 0 0 0 1 9 0 0 0 0 4;0 0 0 0 0 0 0 0 0 9 2 0 0 0;0 0 0 0 0 0 0 0 0 1 9 0 0 4;0 0 0 0 0 0 0 0 0 0 0 9 2 0;0 0 0 0 0 0 0 0 0 0 0 1 9 4;0 0 0 0 0 0 0 0 0 0 0 0 0 9'),numpy.matrix('9,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0;0,9,2,0,0,0,0,0,0,0,0,0,0,0,0,0;0,1,9,0,0,0,0,4,0,0,0,0,0,0,0,0;0,0,0,9,2,0,0,0,0,0,0,0,0,0,0,0;0,0,0,1,9,0,0,4,0,0,0,0,0,0,0,0;0,0,0,0,0,9,2,0,0,0,0,0,0,0,0,0;0,0,0,0,0,1,9,4,0,0,0,0,0,0,0,0;0,0,0,0,0,0,0,9,4,0,4,0,4,0,5,0;0,0,0,0,0,0,0,0,9,2,0,0,0,0,0,0;0,0,0,0,0,0,0,0,1,9,0,0,0,0,0,4;0,0,0,0,0,0,0,0,0,0,9,2,0,0,0,0;0,0,0,0,0,0,0,0,0,0,1,9,0,0,0,4;0,0,0,0,0,0,0,0,0,0,0,0,9,2,0,0;0,0,0,0,0,0,0,0,0,0,0,0,1,9,0,4;0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,2;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9'),numpy.matrix('9 3 0 3 0 3 0 0;0 9 2 0 0 0 0 0;0 1 9 0 0 0 0 5;0 0 0 9 2 0 0 0;0 0 0 1 9 0 0 5;0 0 0 0 0 9 2 0;0 0 0 0 0 1 9 5;0 0 0 0 0 0 0 9'),numpy.matrix('9 1 0 1;1 9 4 0;0 0 9 4;0 0 0 9')]
    for M in A_list:
        for i in finalconnections(M):
            assert i[::-1] in all_paths(M)
        for i in all_paths(M):
            assert i[::-1] in finalconnections(M)
    