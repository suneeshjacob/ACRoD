import numpy
import xml.etree.ElementTree as ET
from src.acrod.jacobian import all_joints_connected_to_the_link

def get_joints_from_urdf(root):
    """
    `get_joints_from_urdf` function extracts a dictionary having information of all the joints in the XML object of the URDF file.

    This function takes the XML object as input argument.

    :param root: XML object of the URDF file (to be given in ElementTree.Element format).
    :type M: ElementTree.Element
    :return: a list of all joint names (from the XML object) as keys, eaching having a tuple of its corresponding type of the joint, parent link name and child link name as values.
    :rtype: dict
    """
    joints_info = []
    for child in root:
        if child.tag == 'joint':
            if child.attrib['type'] in ['revolute', 'continuous', 'prismatic', 'planar']:
                parent_link = None
                child_link = None
                for child2 in child:
                    if child2.tag == 'parent':
                        parent_link = child2.attrib['link']
                    if child2.tag == 'child':
                        child_link = child2.attrib['link']
                joints_info.append((child.attrib['name'], child.attrib['type'], parent_link, child_link,))
    
    return {i[0]:i[1:] for i in sorted(joints_info, key=lambda x:x[0])}

def get_links_from_urdf(root):
    """
    `get_links_from_urdf` function extracts a list having information of names of all the link in the XML object of the URDF file and outputs the list in numpy.array format.

    This function takes the XML object as input argument.

    :param root: XML object of the URDF file (to be given in ElementTree.Element format).
    :type M: ElementTree.Element
    :return: a numpy array containing all the link names (from the XML object).
    :rtype: numpy.ndarray
    """
    links_info = []
    for child in root:
        if child.tag == 'link':
            links_info.append(child.attrib['name'])
    return numpy.unique(sorted(links_info))

def from_urdf_to_matrix(path_to_urdf_file):
    """
    `from_urdf_to_matrix` function takes the URDF file path as the input argument and produces the robot-topology matrix.

    The URDF file is expected to have only one base link and only one end-effector link.

    :param path_to_urdf_file: A valid path for the URDF file.
    :type M: str
    :return: A corresponding robot-topology matrix in numpy array format.
    :rtype: numpy.ndarray
    """
    tree = ET.parse(path_to_urdf_file)
    root = tree.getroot()
    
    all_links = get_links_from_urdf(root)
    all_joints = get_joints_from_urdf(root)
    
    parent_links = set(list(zip(*all_joints.values()))[1])
    child_links = set(list(zip(*all_joints.values()))[2])
    
    base_links = parent_links - child_links
    endeffector_links = child_links - parent_links
    
    base_link = sorted(base_links)[0]
    endeffector_link = sorted(endeffector_links)[0]
    
    n = len(all_links)
    M = 9*numpy.eye(n)
    
    links_list = [base_link] + all_links[numpy.logical_and(all_links != base_link, all_links != endeffector_link)].tolist() + [endeffector_link]
    links_dict = dict(enumerate(links_list))
    links_dict_inverse = {v:k for k,v in enumerate(links_list)}
    
    for i in all_joints:
        ind1 = links_dict_inverse[all_joints[i][1]]
        ind2 = links_dict_inverse[all_joints[i][2]]
        if ind1>ind2:
            temp = ind2
            ind2 = ind1
            ind1 = temp
        if all_joints[i][0] == 'prismatic':
            M[ind1,ind2] = 2
            M[ind2,ind1] = 1
        elif all_joints[i][0] in ['revolute','continuous']:
            M[ind1,ind2] = 1
            M[ind2,ind1] = 1
        elif all_joints[i][0] == 'planar':
            M[ind1,ind2] = 7
    
    return reduce_robottopologymatrix(M)

def reduce_robottopologymatrix(M):
    """
    `reduce_robottopologymatrix` function reduces the robot topology matrix that has unconnected link(s) by removing those link(s).

    This function takes the robot-topology matrix as the input argument (in numpy array format).

    :param M: The robot-topology matrix from which the unconnected links are to be removed off.
    :type M: numpy.ndarray
    :return: A reduced robot-topology matrix by removing the unconnected links.
    :rtype: numpy.ndarray
    """
    listofsumofalljointsconnectedtoeachlink = numpy.array([sum(all_joints_connected_to_the_link(M,i)) for i in range(len(M))])
    lst = listofsumofalljointsconnectedtoeachlink==0
    M_new = M[~lst,:][:,~lst]
    return M_new
