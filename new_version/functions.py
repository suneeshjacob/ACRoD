import numpy
import itertools

def all_joints_connected_to_the_link(M,linknumber):
    """
    all_joints_connected_to_the_link function gives a list of all the joints connected to a given link for a given robot-topology matrix.

    this function takes the robot-topology matrix and the link number as input arguments.

    :param M: robot-topology matrix (of size nxn, to be given in numpy.matrix format).
    :param linknumber: link number (it is to be indexed from 0, not from 1).
    :return: a list of size n-1 items specifying the type of joint with which the given link is connected to every other joint. The order of joints corresponds to the ascending order of the link numbers.
    """
    n=len(M)
    k=list(range(0,linknumber))+list(range(linknumber+1,n))
    kvtemp=[]
    for itemp in k:
        ktemp=[linknumber,itemp]
        ktemp.sort()
        kvtemp.append(int(M[ktemp[0],ktemp[1]]))
    return kvtemp

def all_links_connected_to_the_link(M,linknumber):
    """
    all_links_connected_to_the_link function gives a list of all the links connected to a given link for a given robot-topology matrix.

    this function takes the robot-topology matrix and the link number as input arguments.

    :param M: robot-topology matrix (of size nxn, to be given in numpy.matrix format).
    :param linknumber: link number (it is to be indexed from 0, not from 1).
    :return: a list of items specifying the indices of links with which the given link is connected, in ascending order of the link numbers.
    """
    k = all_joints_connected_to_the_link(M,linknumber)
    k2 = k[:linknumber]+[9]+k[linknumber:]
    k3 = [i for i in range(len(k2)) if ((k2[i]!=0)&(k2[i]!=9))]
    return k3

# Function for getting all possible paths
def all_paths(M):
    """
    all_paths function gives a list of all paths connecting the base link and the end-effector link for a given robot-topology matrix.

    this function takes the robot-topology matrix as input argument.

    :param M: robot-topology matrix (of size nxn, to be given in numpy.matrix format).
    :return: a list of lists of items each corresponding to a path and specifying the sequence of links in that path that are connected from the base link to the end-effector link (the link numbers are indexed from 0, not from 1).
    """
    M = numpy.array(M)
    P = []
    P.append([0]) # Base link number (link numbers starting from zero)
    n = len(M)
    while sum([0 if each_sequence[-1] == n-1 else 1 for each_sequence in P])!=0:
        P_i_list_to_be_removed = []
        for i in range(len(P)):
            l = P[i][-1]
            if l!=n-1:
                L = all_links_connected_to_the_link(M,l)
                P_i_list_to_be_removed.append(i)
                for j in L:
                    if j not in P[i]:
                        all_sub_sequences_of_i = [[P[i][i2],P[i][i2+1]] for i2 in range(len(P[i])-1)]
                        if [l,j] not in all_sub_sequences_of_i:
                            P.append(P[i]+[j])
        for i in sorted(P_i_list_to_be_removed,reverse=True):
            P.pop(i)
    
    return P

def get_all_combinations_of_two_links(M_size):
    output_list = []
    if M_size%2 == 0:
        n = int(M_size/2)
    else:
        n = int((M_size-1)/2) + 1
    
    for i in range(1,n):
        all_combinations_of_i = list(itertools.combinations(range(M_size),i))
        for j in all_combinations_of_i:
            other_set = set(range(M_size))-set(j)
            output_list.append([set(j),other_set])
    
    if M_size%2 == 0:
        list_of_all_combinations = list(itertools.combinations(range(M_size),n))
        list_with_redundancy = []
        for i in list_of_all_combinations:
            other_set = set(range(M_size))-set(i)
            list_with_redundancy.append([set(i),other_set])
        confirmed_list = []
        
        for i in list_with_redundancy:
            if i[0] not in confirmed_list and i[1] not in confirmed_list:
                confirmed_list.append(i[0])
                confirmed_list.append(i[1])
                output_list.append(i)
    return output_list

def graph_adjacency_matrix_from_robot_topology_matrix(M):
    A = M.copy()
    B = numpy.matrix([[A[i,j] if j>i else 0 for j in range(A.shape[1])] for i in range(A.shape[0])])
    C = B + B.T
    return C

# Function for getting information about superfluous DOF
def superfluous(M):
    S = []
    if numpy.sum(M==4) > 2:
        C = get_all_combinations_of_two_links(len(M))
        for c in C:
            c1 = list(sorted(list(c[0])))
            c2 = list(sorted(list(c[1])))
            indices_list = c1 + c2
            A = graph_adjacency_matrix_from_robot_topology_matrix(M)
            D = (A[indices_list,:][:,indices_list])[:len(c1),len(c1):]



    return S
