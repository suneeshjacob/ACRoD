import numpy
import sympy
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
    if numpy.sum(M==4) >= 2:
        C = get_all_combinations_of_two_links(len(M))
        for c in C:
            c1 = list(sorted(list(c[0])))
            c2 = list(sorted(list(c[1])))
            indices_list = c1 + c2
            A = graph_adjacency_matrix_from_robot_topology_matrix(M)
            D = (A[indices_list,:][:,indices_list])[:len(c1),len(c1):]
            if numpy.sum(D==4) == 2 and numpy.sum(D==0) == numpy.prod(D.shape)-2:
                if 0 in c1 and len(M)-1 in c1:
                    c_be = c1
                elif 0 in c2 and len(M)-1 in c2:
                    c_be = c2
                else:
                    uncontrollable = True
                    return uncontrollable
                c1_a = numpy.array(c1)
                c2_a = numpy.array(c2)
                spherical_indices = numpy.where(D==4)
                ijkl = numpy.array([c1_a[spherical_indices[0]],c2_a[spherical_indices[1]]]).T
                ij,kl = ijkl
                if ij[0] in c_be:
                    i,j = ij
                else:
                    j,i = ij
                if kl[1] in c_be:
                    k,l = kl
                else:
                    l,k = kl

                S.append([c_be,[(i,j),(k,l)]])



    return S

class jacobian(object):
    def __init__(self, M, robot_type = 'spatial'):
        self.M = M
        self.type = robot_type
        self.P = None
        self.P_tilde = None
        self.P_tilde_omega = None
        self.is_serial = None
    def get_all_paths(self):
        M = self.M
        P = all_paths(M)
        self.P = P
        if len(P) == 1:
            self.is_serial = True
        else:
            self.is_serial = False
    def get_independent_paths(self):
        P = self.P
        M = self.M
        P_tilde, P_tilde_omega, indices_P_tilde, indices_P_tilde_omega = from_P_to_P_tilde(P,M)
        self.P_tilde = P_tilde
        self.P_tilde_omega = P_tilde_omega
        self.indices_P_tilde = indices_P_tilde
        self.indices_P_tilde_omega = indices_P_tilde_omega
    def execute_equations(self):
        M = self.M
        P = self.P
        indices_P_tilde = self.indices_P_tilde
        indices_P_tilde_omega = self.indices_P_tilde_omega
        current_variables = []
        executions = [f"a_x = sympy.symbols(r'a_x')", f"a_y = sympy.symbols(r'a_y')", f"a_z = sympy.symbols(r'a_z')", f"a = sympy.Matrix([[a_x],[a_y],[a_z]])", "zero = sympy.Matrix([[0],[0],[0]])"]
        linear_velocities = []
        angular_velocities = []
        for i in range(len(P)):
            if i in indices_P_tilde:
                if i in indices_P_tilde_omega:
                    linear_velocity, angular_velocity, executions_list, current_variables = vel_path(M, P[i], current_variables)
                    linear_velocities.append(linear_velocity)
                    angular_velocities.append(angular_velocity)
                else:
                    linear_velocity, _, executions_list, current_variables = vel_path(M, P[i], current_variables)
                    linear_velocities.append(linear_velocity)
                executions += executions_list
        
        superfluous_info = superfluous(M)
        superfluous_equations_executions = []
        if len(superfluous_info) > 0:
            for i in range(len(superfluous_info)):
                current_superfluous_info = superfluous_info[i]
                L_s = current_superfluous_info[1][0][1]
                path_containing_superfluous_link = [i for i in P if L_s in i][0]
                truncated_path = path_containing_superfluous_link[:path_containing_superfluous_link.index(L_s)+1]
                _, superfluous_link_angular_velocity, executions_list, current_variables = vel_path(M, truncated_path, current_variables)
                executions += executions_list
                #(i_u, j_u), (k_u, l_u) = current_superfluous_info[1]
                i_s, j_s = sorted(current_superfluous_info[1][0])
                k_s, l_s = sorted(current_superfluous_info[1][1])
                superfluous_equations_executions.append('('+superfluous_link_angular_velocity+f').dot(r_{i_s}_{j_s}-r_{k_s}_{l_s})')

        import sympy
        for i in executions:
            exec(i, locals())
        
        linear_velocities_expressions = []
        angular_velocities_expressions = []
        superfluous_equations_expressions = []
        for i in linear_velocities:
            linear_velocities_expressions.append(eval(i))
        for i in angular_velocities:
            angular_velocities_expressions.append(eval(i))
        for i in superfluous_equations_executions:
            superfluous_equations_expressions.append(eval(i))
        #linear_velocities_expressions = [eval(i,locals()) for i in linear_velocities]
        #angular_velocities_expressions = [eval(i) for i in angular_velocities]
        #superfluous_equations_expressions = [eval(i) for i in superfluous_equations_executions]
        J_matrix = sympy.Matrix([linear_velocities_expressions[0], angular_velocities_expressions[0]])
        A_matrix = sympy.Matrix([linear_velocities_expressions[i+1]-linear_velocities_expressions[0] for i in range(len(linear_velocities_expressions)-1)]+[angular_velocities_expressions[i+1]-angular_velocities_expressions[0] for i in range(len(angular_velocities_expressions)-1)]+superfluous_equations_expressions)
        
        active_jointvelocities, passive_jointvelocities = get_jointvelocities_list(M)
        active = []
        passive = []
        for i in active_jointvelocities:
            active.append(eval(i))
        for i in passive_jointvelocities:
            passive.append(eval(i))

        #active = [eval(i) for i in active_jointvelocities]
        #passive = [eval(i) for i in passive_jointvelocities]
        
        J_a = J_matrix.jacobian(active)
        J_p = J_matrix.jacobian(passive)
        A_a = A_matrix.jacobian(active)
        A_p = A_matrix.jacobian(passive)

        self.J_a = J_a
        self.J_p = J_a
        self.A_a = A_a
        self.A_p = A_p

        decision_variables_str = sum(get_variables_list(M).values(),[])
        decision_variables = []
        for i in decision_variables_str:
            decision_variables.append(eval(i))
        #decision_variables = [eval(i) for i in decision_variables_str]
        endeffector_variables = [eval('a_x'), eval('a_y'), eval('a_z')]

        J_a_func = sympy.lambdify([endeffector_variables, decision_variables],J_a)
        J_p_func = sympy.lambdify([endeffector_variables, decision_variables],J_p)
        A_a_func = sympy.lambdify([endeffector_variables, decision_variables],A_a)
        A_p_func = sympy.lambdify([endeffector_variables, decision_variables],A_p)

        return J_a_func, J_p_func, A_a_func, A_p_func, active_jointvelocities, passive_jointvelocities, decision_variables_str




def vel_path(M, path, available_variables):

    executions_list = []
    linear_velocity = ''
    angular_velocity = ''

    for index in range(len(path)-1):
        i,j = path[index:index+2]
        if i < j:
            i_s, j_s = i, j
            direction = 'ascending'
        else:
            i_s, j_s = j, i
            direction = 'descending'
        

        A = graph_adjacency_matrix_from_robot_topology_matrix(M)
        joint_type = A[i,j]
        if joint_type == 1:
            if f'{i_s}_{j_s}' not in available_variables:
                executions_list.append(f"r_{i_s+1}_{j_s+1}_x = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})x}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_y = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})y}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_z = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})z}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1} = sympy.Matrix([[r_{i_s+1}_{j_s+1}_x],[r_{i_s+1}_{j_s+1}_y],[r_{i_s+1}_{j_s+1}_z]])")
                executions_list.append(f"beta_{i_s+1}_{j_s+1} = sympy.symbols(r'\\beta_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"phi_{i_s+1}_{j_s+1} = sympy.symbols(r'\phi_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"n_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.cos(phi_{i_s+1}_{j_s+1})],[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.sin(phi_{i_s+1}_{j_s+1})],[sympy.cos(beta_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"thd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{\\theta}}_{{({i_s+1}\,{j_s+1})}}')")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}.cross(a-r_{i_s+1}_{j_s+1})'
                angular_velocity += f'-thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
            else:
                linear_velocity += f'+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}.cross(a-r_{i_s+1}_{j_s+1})'
                angular_velocity += f'+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
        elif joint_type == 2:
            if f'{i_s}_{j_s}' not in available_variables:
                executions_list.append(f"beta_{i_s+1}_{j_s+1} = sympy.symbols(r'\\beta_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"phi_{i_s+1}_{j_s+1} = sympy.symbols(r'\phi_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"n_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.cos(phi_{i_s+1}_{j_s+1})],[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.sin(phi_{i_s+1}_{j_s+1})],[sympy.cos(beta_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"dd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{d}}_{{({i_s+1}\,{j_s+1})}}')")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-dd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
                angular_velocity += f'-zero'
            else:
                linear_velocity += f'+dd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
                angular_velocity += f'+zero'
        elif joint_type == 3:
            if f'{i_s}_{j_s}' not in available_variables:
                executions_list.append(f"r_{i_s+1}_{j_s+1}_x = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})x}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_y = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})y}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_z = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})z}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1} = sympy.Matrix([[r_{i_s+1}_{j_s+1}_x],[r_{i_s+1}_{j_s+1}_y],[r_{i_s+1}_{j_s+1}_z]])")
                executions_list.append(f"beta_{i_s+1}_{j_s+1} = sympy.symbols(r'\\beta_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"phi_{i_s+1}_{j_s+1} = sympy.symbols(r'\phi_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"n_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.cos(phi_{i_s+1}_{j_s+1})],[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.sin(phi_{i_s+1}_{j_s+1})],[sympy.cos(beta_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"thd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{\\theta}}_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"dd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{d}}_{{({i_s+1}\,{j_s+1})}}')")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}.cross(a-r_{i_s+1}_{j_s+1}) - dd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
                angular_velocity += f'-thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
            else:
                linear_velocity += f'+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}.cross(a-r_{i_s+1}_{j_s+1}) + dd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
                angular_velocity += f'+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
        elif joint_type == 4:
            if f'{i_s}_{j_s}' not in available_variables:
                executions_list.append(f"r_{i_s+1}_{j_s+1}_x = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})x}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_y = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})y}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_z = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})z}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1} = sympy.Matrix([[r_{i_s+1}_{j_s+1}_x],[r_{i_s+1}_{j_s+1}_y],[r_{i_s+1}_{j_s+1}_z]])")
                executions_list.append(f"w_{i_s+1}_{j_s+1}_x = sympy.symbols(r'\omega_{{({i_s+1}\,{j_s+1})x}}')")
                executions_list.append(f"w_{i_s+1}_{j_s+1}_y = sympy.symbols(r'\omega_{{({i_s+1}\,{j_s+1})y}}')")
                executions_list.append(f"w_{i_s+1}_{j_s+1}_z = sympy.symbols(r'\omega_{{({i_s+1}\,{j_s+1})z}}')")
                executions_list.append(f"w_{i_s+1}_{j_s+1} = sympy.Matrix([[w_{i_s+1}_{j_s+1}_x],[w_{i_s+1}_{j_s+1}_y],[w_{i_s+1}_{j_s+1}_z]])")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-w_{i_s+1}_{j_s+1}.cross(a-r_{i_s+1}_{j_s+1})'
                angular_velocity += f'-w_{i_s+1}_{j_s+1}'
            else:
                linear_velocity += f'+w_{i_s+1}_{j_s+1}.cross(a-r_{i_s+1}_{j_s+1})'
                angular_velocity += f'+w_{i_s+1}_{j_s+1}'
        elif joint_type == 5:
            if f'{i_s}_{j_s}' not in available_variables:
                executions_list.append(f"r_{i_s+1}_{j_s+1}_x = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})x}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_y = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})y}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_z = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})z}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1} = sympy.Matrix([[r_{i_s+1}_{j_s+1}_x],[r_{i_s+1}_{j_s+1}_y],[r_{i_s+1}_{j_s+1}_z]])")
                executions_list.append(f"beta_{i_s+1}_{j_s+1} = sympy.symbols(r'\\beta_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"phi_{i_s+1}_{j_s+1} = sympy.symbols(r'\phi_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"delta_{i_s+1}_{j_s+1} = sympy.symbols(r'\delta_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"alpha_{i_s+1}_{j_s+1} = sympy.atan(-1/(sympy.symbols(r'sympy.tan(\\beta_{{({i_s+1}\,{j_s+1})}})*sympy.cos(\delta_{{({i_s+1}\,{j_s+1})}}-\phi_{{({i_s+1}\,{j_s+1})}})')))")
                executions_list.append(f"n_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.cos(phi_{i_s+1}_{j_s+1})],[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.sin(phi_{i_s+1}_{j_s+1})],[sympy.cos(beta_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"m_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.sin(alpha_{i_s+1}_{j_s+1})*sympy.cos(delta_{i_s+1}_{j_s+1})],[sympy.sin(alpha_{i_s+1}_{j_s+1})*sympy.sin(delta_{i_s+1}_{j_s+1})],[sympy.cos(alpha_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"thd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{\\theta}}_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"gamd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{\gamma}}_{{({i_s+1}\,{j_s+1})}}')")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-(gamd_{i_s+1}_{j_s+1}*m_{i_s+1}_{j_s+1}+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}).cross(a-r_{i_s+1}_{j_s+1})'
                angular_velocity += f'-(gamd_{i_s+1}_{j_s+1}*m_{i_s+1}_{j_s+1}+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1})'
            else:
                linear_velocity += f'+(gamd_{i_s+1}_{j_s+1}*m_{i_s+1}_{j_s+1}+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}).cross(a-r_{i_s+1}_{j_s+1})'
                angular_velocity += f'+(gamd_{i_s+1}_{j_s+1}*m_{i_s+1}_{j_s+1}+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1})'
        elif joint_type == 6:
            if f'{i_s}_{j_s}' not in available_variables:
                executions_list.append(f"r_{i_s+1}_{j_s+1}_x = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})x}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_y = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})y}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_z = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})z}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1} = sympy.Matrix([[r_{i_s+1}_{j_s+1}_x],[r_{i_s+1}_{j_s+1}_y],[r_{i_s+1}_{j_s+1}_z]])")
                executions_list.append(f"beta_{i_s+1}_{j_s+1} = sympy.symbols(r'\\beta_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"phi_{i_s+1}_{j_s+1} = sympy.symbols(r'\phi_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"p_{i_s+1}_{j_s+1} = sympy.symbols(r'p_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"n_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.cos(phi_{i_s+1}_{j_s+1})],[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.sin(phi_{i_s+1}_{j_s+1})],[sympy.cos(beta_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"thd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{\\theta}}_{{({i_s+1}\,{j_s+1})}}')")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-thd_{i_s+1}_{j_s+1}*(n_{i_s+1}_{j_s+1}.cross(a-r_{i_s+1}_{j_s+1})+(p_{i_s+1}_{j_s+1}/(2*sympy.pi))*n_{i_s+1}_{j_s+1})'
                angular_velocity += f'-thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
            else:
                linear_velocity += f'+thd_{i_s+1}_{j_s+1}*(n_{i_s+1}_{j_s+1}.cross(a-r_{i_s+1}_{j_s+1})+(p_{i_s+1}_{j_s+1}/(2*sympy.pi))*n_{i_s+1}_{j_s+1})'
                angular_velocity += f'+thd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
        elif joint_type == 7:
            if f'{i_s}_{j_s}' not in available_variables:
                executions_list.append(f"r_{i_s+1}_{j_s+1}_x = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})x}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_y = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})y}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1}_z = sympy.symbols(r'r_{{({i_s+1}\,{j_s+1})z}}')")
                executions_list.append(f"r_{i_s+1}_{j_s+1} = sympy.Matrix([[r_{i_s+1}_{j_s+1}_x],[r_{i_s+1}_{j_s+1}_y],[r_{i_s+1}_{j_s+1}_z]])")
                executions_list.append(f"beta_{i_s+1}_{j_s+1} = sympy.symbols(r'\\beta_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"phi_{i_s+1}_{j_s+1} = sympy.symbols(r'\phi_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"n_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.cos(phi_{i_s+1}_{j_s+1})],[sympy.sin(beta_{i_s+1}_{j_s+1})*sympy.sin(phi_{i_s+1}_{j_s+1})],[sympy.cos(beta_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"dd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{d}}_{{({i_s+1}\,{j_s+1})}}')")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-dd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
                angular_velocity += f'-zero'
            else:
                linear_velocity += f'+dd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
                angular_velocity += f'+zero'
    return linear_velocity, angular_velocity, executions_list, available_variables

def get_jointvelocities_list(M):
    active_jointvelocities_list = []
    passive_jointvelocities_list = []
    for i in range(len(M)):
        for j in range(i+1,len(M)):
            if M[i,j] in [1,2]:
                if M[i,j] == 1:
                    if M[j,i] == 1:
                        active_jointvelocities_list.append(f'thd_{i+1}_{j+1}')
                    elif M[j,i] == 0:
                        passive_jointvelocities_list.append(f'thd_{i+1}_{j+1}')
                elif M[i,j] == 2:
                    if M[j,i] == 1:
                        active_jointvelocities_list.append(f'dd_{i+1}_{j+1}')
                    elif M[j,i] == 0:
                        passive_jointvelocities_list.append(f'dd_{i+1}_{j+1}')
            else:
                if M[i,j] == 3:
                    passive_jointvelocities_list.append(f'thd_{i+1}_{j+1}')
                    passive_jointvelocities_list.append(f'dd_{i+1}_{j+1}')
                elif M[i,j] == 4:
                    passive_jointvelocities_list.append(f'w_{i+1}_{j+1}_x')
                    passive_jointvelocities_list.append(f'w_{i+1}_{j+1}_y')
                    passive_jointvelocities_list.append(f'w_{i+1}_{j+1}_z')
                elif M[i,j] == 5:
                    passive_jointvelocities_list.append(f'thd_{i+1}_{j+1}')
                    passive_jointvelocities_list.append(f'gamd_{i+1}_{j+1}')
                elif M[i,j] == 6:
                    passive_jointvelocities_list.append(f'thd_{i+1}_{j+1}')
                elif M[i,j] == 7:
                    passive_jointvelocities_list.append(f'dd_{i+1}_{j+1}')
    
    return active_jointvelocities_list, passive_jointvelocities_list

def from_P_to_P_tilde(P,M):
    V_i_j_vector = []
    Cv = numpy.reshape(numpy.matrix(''),(0,0))
    for i in range(len(P)):
        Cv = numpy.r_[Cv,numpy.zeros((1,len(V_i_j_vector)))]
        for j in range(len(P[i])-1):
            V_i_j_element_unsorted = [P[i][j],P[i][j+1]]
            V_i_j_element_sorted = list(sorted(V_i_j_element_unsorted))
            if V_i_j_element_sorted == V_i_j_element_unsorted:
                sortflag = 1
            elif V_i_j_element_sorted == V_i_j_element_unsorted[::-1]:
                sortflag = -1
            if V_i_j_element_sorted not in V_i_j_vector:
                V_i_j_vector.append(V_i_j_element_sorted)
                Cv = numpy.c_[Cv,numpy.r_[numpy.zeros((Cv.shape[0]-1,1)),numpy.matrix(sortflag)]]
            elif V_i_j_element_sorted in V_i_j_vector:
                ind = V_i_j_vector.index(V_i_j_element_sorted)
                Cv[i,ind]= sortflag
    _,independent_path_indices = sympy.Matrix(Cv).T.rref()
    booleans_for_nonprismatic_joints = [False if M[i[0],i[1]]==2 else True for i in V_i_j_vector]
    Comega = Cv[independent_path_indices,:][:,booleans_for_nonprismatic_joints]
    _,independent_omega_path_indices = sympy.Matrix(Comega.T).rref()
    P_tilde = [P[i] for i in range(len(P)) if i in independent_path_indices]
    P_tilde_omega = [P[i] for i in range(len(P)) if i in independent_omega_path_indices]
    return [P_tilde,P_tilde_omega,independent_path_indices,independent_omega_path_indices]


def get_variables_list(M):
    variables_dict = {}
    for i in range(len(M)):
        for j in range(i+1,len(M)):
            current_variables_list = []
            if M[i,j] == 0:
                pass
            elif M[i,j] == 1:
                current_variables_list.append(f'r_{i+1}_{j+1}_x')
                current_variables_list.append(f'r_{i+1}_{j+1}_y')
                current_variables_list.append(f'r_{i+1}_{j+1}_z')
                current_variables_list.append(f'beta_{i+1}_{j+1}')
                current_variables_list.append(f'phi_{i+1}_{j+1}')
            elif M[i,j] == 2:
                current_variables_list.append(f'beta_{i+1}_{j+1}')
                current_variables_list.append(f'phi_{i+1}_{j+1}')
            elif M[i,j] == 3:
                current_variables_list.append(f'r_{i+1}_{j+1}_x')
                current_variables_list.append(f'r_{i+1}_{j+1}_y')
                current_variables_list.append(f'r_{i+1}_{j+1}_z')
                current_variables_list.append(f'beta_{i+1}_{j+1}')
                current_variables_list.append(f'phi_{i+1}_{j+1}')
            elif M[i,j] == 4:
                current_variables_list.append(f'r_{i+1}_{j+1}_x')
                current_variables_list.append(f'r_{i+1}_{j+1}_y')
                current_variables_list.append(f'r_{i+1}_{j+1}_z')
            elif M[i,j] == 5:
                current_variables_list.append(f'r_{i+1}_{j+1}_x')
                current_variables_list.append(f'r_{i+1}_{j+1}_y')
                current_variables_list.append(f'r_{i+1}_{j+1}_z')
                current_variables_list.append(f'beta_{i+1}_{j+1}')
                current_variables_list.append(f'phi_{i+1}_{j+1}')
                current_variables_list.append(f'delta_{i+1}_{j+1}')
            elif M[i,j] == 6:
                current_variables_list.append(f'r_{i+1}_{j+1}_x')
                current_variables_list.append(f'r_{i+1}_{j+1}_y')
                current_variables_list.append(f'r_{i+1}_{j+1}_z')
                current_variables_list.append(f'beta_{i+1}_{j+1}')
                current_variables_list.append(f'phi_{i+1}_{j+1}')
                current_variables_list.append(f'p_{i+1}_{j+1}')
            elif M[i,j] == 7:
                current_variables_list.append(f'beta_{i+1}_{j+1}')
                current_variables_list.append(f'phi_{i+1}_{j+1}')
            if len(current_variables_list) > 0:
                variables_dict[(i,j)] = current_variables_list
    return variables_dict