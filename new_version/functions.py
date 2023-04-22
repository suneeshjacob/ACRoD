import numpy

def all_joints_connected_to_the_link(M,linknumber):
    """
    all_joints_connected_to_the_link function gives a list of all the joints connected to a given link for a given robot-topology matrix.

    this function takes the robot-topology matrix and the link number as input arguments.

    :param M: robot-topology matrix (of size nxn, to be given in numpy.matrix format).
    :param linknumber: link number (it is to be indexed from 0, not from 1).
    :return: a list of size n-1 items specifying the type of joint with which the given link is connected to every other joint.
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
    k = all_joints_connected_to_the_link(M,linknumber)
    k2 = k[:linknumber]+[9]+k[linknumber:]
    k3 = [i for i in range(len(k2)) if ((k2[i]!=0)&(k2[i]!=9))]
    return k3

# Function for getting all possible paths
def all_paths(M):
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

# Function for getting information about superfluous DOF
