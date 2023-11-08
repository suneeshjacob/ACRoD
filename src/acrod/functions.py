import numpy
import sympy
import itertools

def all_joints_connected_to_the_link(M,linknumber):
    """
    all_joints_connected_to_the_link function gives a list of all the joints connected to a given link for a given robot-topology matrix.

    this function takes the robot-topology matrix and the link number as input arguments.

    :param M: robot-topology matrix (of size nxn, to be given in numpy.array format).
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

    :param M: robot-topology matrix (of size nxn, to be given in numpy.array format).
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

    :param M: robot-topology matrix (of size nxn, to be given in numpy.array format).
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

def get_all_combinations_of_two_parts_of_manipulator(M_size):
    """
    get_all_combinations_of_two_parts_of_manipulator function gives all possible combinations of splitting the given set of links into two parts.

    this function takes the number of links (which is the same as the size of the robot-topology matrix) as input argument.

    :param M_size: The number of links of the robot.
    :return: the entire possible list of items each containing the entire links grouped into two parts.
    """
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
    """
    graph_adjacency_matrix_from_robot_topology_matrix function converts a robot-topology matrix to its graph adjacency matrix. by doing so, it would no longer have the information to identify the actuating joints.

    This function takes robot-topology matrix as input argument.

    :param M: robot-topology matrix.
    :return: the corresponding graph adjacency matrix.
    """
    A = M.copy()
    B = numpy.array([[A[i,j] if j>i else 0 for j in range(A.shape[1])] for i in range(A.shape[0])])
    C = B + B.T
    return C

# Function for getting information about superfluous DOF
def superfluous(M):
    """
    superfluous function gives the information related to superfluous DOF if there are any, from the robot-topology matrix.

    This function takes robot-topology matrix as input argument.

    :param M: robot-topology matrix.
    :return: a list of pieces of information regarding superfluous DOF. each piece of superfluous DOF would be of the format of [c_be, [(i,j),(k,l)]], where c_be is the part of links that has both base and the end-effector links, and [(i,j),(k,l)] are the points indices of two spherical joints corresponding to the superfluous DOF, in which the links j, k lie on the other part than c_be. if there are no superfluous DOF then it returns an empty list.
    """
    S = []
    if numpy.sum(M==4) >= 2:
        C = get_all_combinations_of_two_parts_of_manipulator(len(M))
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
    """
    jacobian class takes the robot-topology matrix as the input. additionally, it takes the robot_type argument (optional) as either 'spatial' or 'planar'. by default, it is 'spatial'.

    attributes:
    M: robot-topology matrix.
    type: robot_type (either 'planar' or 'spatial').
    P: list of all paths.
    P_tilde: list of independent paths for linear velocities.
    P_tilde_omega: list of independent paths for angular velocitites.
    is_serial: boolean, mentioning whether the manipulator is a serial manipulator or not a serial manipulator. it is initialised with None before determining whether the manipulator is serial or not.
    Ja: J_a in symbolic form.
    Jp: J_p in symbolic form.
    Aa: A_a in symbolic form.
    Ap: A_p in symbolic form.
    Ja_func: J_a in Python function form.
    Jp_func: J_p in Python function form.
    Aa_func: A_a in Python function form.
    Ap_func: A_p in Python function form.
    active_joint_velocities: list of active joint velocities, each in string format.
    passive_joint_velocities: list of passive joint velocities, each in string format.
    parameters: list of parameters (other than end-effector parameters), each in string format.
    active_joint_velocities_symbolic: active joint velocities in symbolic form.
    passive_joint_velocities_symbolic: passive joint velocities in symbolic form.
    parameters_symbolic: list of parameters (other than end-effector parameters) in string format.
    endeffector_variables_symbolic: end-effector parameters in symbolic form.
    superfluous_dof_information: list containing information of superfluous DOF existing in the robot.
    """
    def __init__(self, M, robot_type = 'spatial'):
        self.M = M
        self.type = robot_type
        self.P = None
        self.P_tilde = None
        self.P_tilde_omega = None
        self.is_serial = None
        self.indices_P_tilde = None
        self.indices_P_tilde_omega = None
        self.Ja = None
        self.Jp = None
        self.Aa = None
        self.Ap = None
        self.Ja_func = None
        self.Jp_func = None
        self.Aa_func = None
        self.Ap_func = None
        self.active_joint_velocities = None
        self.passive_joint_velocities = None
        self.parameters = None
        self.active_joint_velocities_symbolic = None
        self.passive_joint_velocities_symbolic = None
        self.parameters_symbolic = None
        self.endeffector_variables_symbolic = None
        self.superfluous_dof_information = None
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
        robot_type = self.type
        indices_P_tilde = self.indices_P_tilde
        indices_P_tilde_omega = self.indices_P_tilde_omega
        current_variables = []
        if robot_type == 'spatial':
            executions = [f"a_x = sympy.symbols(r'a_x')", f"a_y = sympy.symbols(r'a_y')", f"a_z = sympy.symbols(r'a_z')", f"a = sympy.Matrix([[a_x],[a_y],[a_z]])", "zero = sympy.Matrix([[0],[0],[0]])"]
        elif robot_type == 'planar':
            executions = [f"a_x = sympy.symbols(r'a_x')", f"a_y = sympy.symbols(r'a_y')", f"a = sympy.Matrix([[a_x],[a_y]])", "zero_1D = sympy.Matrix([[0]])"]
        linear_velocities = []
        angular_velocities = []
        for i in range(len(P)):
            if i in indices_P_tilde:
                if i in indices_P_tilde_omega:
                    if robot_type == 'spatial':
                        linear_velocity, angular_velocity, executions_list, current_variables = vel_path(M, P[i], current_variables)
                    elif robot_type == 'planar':
                        linear_velocity, angular_velocity, executions_list, current_variables = vel_path_planar(M, P[i], current_variables)
                    linear_velocities.append(linear_velocity)
                    angular_velocities.append(angular_velocity)
                else:
                    if robot_type == 'spatial':
                        linear_velocity, _, executions_list, current_variables = vel_path(M, P[i], current_variables)
                    elif robot_type == 'planar':
                        linear_velocity, _, executions_list, current_variables = vel_path_planar(M, P[i], current_variables)
                    linear_velocities.append(linear_velocity)
                executions += executions_list
        
        superfluous_equations_executions = []
        if robot_type == 'spatial':
            superfluous_info = superfluous(M)
            if len(superfluous_info) > 0:
                self.superfluous_dof_information = superfluous_info
                for i in range(len(superfluous_info)):
                    current_superfluous_info = superfluous_info[i]
                    L_s = current_superfluous_info[1][0][1]
                    path_containing_superfluous_link = [i for i in P if L_s in i][0]
                    truncated_path = path_containing_superfluous_link[:path_containing_superfluous_link.index(L_s)+1]
                    _, superfluous_link_angular_velocity, executions_list, current_variables = vel_path(M, truncated_path, current_variables)
                    executions += executions_list

                    if superfluous_link_angular_velocity[0] == '+':
                        superfluous_link_angular_velocity = superfluous_link_angular_velocity[1:]
                    
                    i_s, j_s = sorted(current_superfluous_info[1][0])
                    k_s, l_s = sorted(current_superfluous_info[1][1])
                    superfluous_equations_executions.append('('+superfluous_link_angular_velocity+f').dot(r_{i_s+1}_{j_s+1}-r_{k_s+1}_{l_s+1})')

        import sympy
        for i in executions:
            exec(i, locals())
        
        linear_velocities_expressions = []
        angular_velocities_expressions = []
        superfluous_equations_expressions = []
        for i in linear_velocities:
            if i[0] == '+':
                linear_velocities_expressions.append(eval(i[1:]))
            else:
                linear_velocities_expressions.append(eval(i))
        for i in angular_velocities:
            if i[0] == '+':
                angular_velocities_expressions.append(eval(i[1:]))
            else:
                angular_velocities_expressions.append(eval(i))
        for i in superfluous_equations_executions:
            superfluous_equations_expressions.append(eval(i))

        if len(angular_velocities_expressions) == 0:
            if robot_type == 'planar':
                zero_for_planar = sympy.Matrix([[0]])
                angular_velocities_expressions.append(zero_for_planar)
            elif robot_type == 'spatial':
                zero_for_spatial = sympy.Matrix([[0],[0],[0]])
                angular_velocities_expressions.append(zero_for_spatial)
        
        J_matrix = sympy.Matrix([linear_velocities_expressions[0], angular_velocities_expressions[0]])
        if len(linear_velocities_expressions)>1 and len(angular_velocities_expressions)==1:
            A_matrix = sympy.Matrix([linear_velocities_expressions[i+1]-linear_velocities_expressions[0] for i in range(len(linear_velocities_expressions)-1)]+angular_velocities_expressions+superfluous_equations_expressions)
        else:
            A_matrix = sympy.Matrix([linear_velocities_expressions[i+1]-linear_velocities_expressions[0] for i in range(len(linear_velocities_expressions)-1)]+[angular_velocities_expressions[i+1]-angular_velocities_expressions[0] for i in range(len(angular_velocities_expressions)-1)]+superfluous_equations_expressions)

        active_jointvelocities, passive_jointvelocities = get_jointvelocities_list(M)
        active = []
        passive = []
        for i in active_jointvelocities:
            active.append(eval(i))
        for i in passive_jointvelocities:
            passive.append(eval(i))




        
        Ja = J_matrix.jacobian(active)
        self.Ja = Ja

        if self.is_serial == False:
            Jp = J_matrix.jacobian(passive)
            Aa = A_matrix.jacobian(active)
            Ap = A_matrix.jacobian(passive)

            self.Jp = Jp
            self.Aa = Aa
            self.Ap = Ap

        if robot_type == 'spatial':
            decision_variables_str = sum(get_variables_list(M).values(),[])
            endeffector_variables = [eval('a_x'), eval('a_y'), eval('a_z')]
        elif robot_type == 'planar':
            decision_variables_str = sum(get_variables_list_planar(M).values(),[])
            endeffector_variables = [eval('a_x'), eval('a_y')]
        decision_variables = []
        for i in decision_variables_str:
            decision_variables.append(eval(i))

        self.parameters = decision_variables_str
        self.parameters_symbolic = sympy.Matrix(decision_variables)

        self.endeffector_variables_symbolic = sympy.Matrix(endeffector_variables)

        Ja_func = sympy.lambdify([endeffector_variables, decision_variables],Ja)
        self.Ja_func = Ja_func

        if self.is_serial == False:
            Jp_func = sympy.lambdify([endeffector_variables, decision_variables],Jp)
            Aa_func = sympy.lambdify([endeffector_variables, decision_variables],Aa)
            Ap_func = sympy.lambdify([endeffector_variables, decision_variables],Ap)

            self.Jp_func = Jp_func
            self.Aa_func = Aa_func
            self.Ap_func = Ap_func
        else:
            Jp_func = None
            Aa_func = None
            Ap_func = None

        self.active_joint_velocities = active_jointvelocities
        self.active_joint_velocities_symbolic = sympy.Matrix(active)
        if self.is_serial == False:
            self.passive_joint_velocities = passive_jointvelocities
            self.passive_joint_velocities_symbolic = sympy.Matrix(passive)

        
        

        return Ja_func, Jp_func, Aa_func, Ap_func, active_jointvelocities, passive_jointvelocities, decision_variables_str

    def process_functions(self):
        self.get_all_paths()
        self.get_independent_paths()
        self.execute_equations()

    def get_jacobian_function(self, MoorePenrose = False):
        self.process_functions()
        Ja = self.Ja_func
        if self.is_serial == False:
            Jp = self.Jp_func
            Aa = self.Aa_func
            Ap = self.Ap_func
            if MoorePenrose == True:
                J = lambda a,x: numpy.array(Ja(a,x)) - numpy.matmul(numpy.matmul(numpy.array(Jp(a,x)),numpy.linalg.pinv(numpy.array(Ap(a,x)))),numpy.array(Aa(a,x)))
            else:
                J = lambda a,x: numpy.array(Ja(a,x)) - numpy.matmul(numpy.matmul(numpy.array(Jp(a,x)),numpy.linalg.inv(numpy.array(Ap(a,x)))),numpy.array(Aa(a,x)))
        else:
            J = lambda a,x: numpy.array(Ja(a,x))
        return J

def vel_path_planar(M, path, available_variables):
    """
    vel_path_planar function processes the strings to be executed for symbolic variabes and velocity expressions of a given path by using the robot-topology matrix, and it also updates the available_variables (so that only those variables that are unavailable are added, thereby avoiding redundancy). this is for planar manipulators.

    this function takes robot-topology matrix, the path and the available variables list (string format) as input arguments.

    :param M: robot-topology matrix.
    :param path: a given path (containing link numbers in order) in list format.
    :param available_variables: a list of available variables (each of them being in string format).
    :return: the expression for linear velocity (in string format, to be executed), the expression for angular velocity (in string format, to be executed), a list of expressions for symbolic variables (in string format, to be executed), and the updated available_variables.
    """
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
                executions_list.append(f"r_{i_s+1}_{j_s+1} = sympy.Matrix([[r_{i_s+1}_{j_s+1}_x],[r_{i_s+1}_{j_s+1}_y]])")
                executions_list.append(f"phi_{i_s+1}_{j_s+1} = sympy.symbols(r'\phi_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"n_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.cos(phi_{i_s+1}_{j_s+1})],[sympy.sin(phi_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"thd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{\\theta}}_{{({i_s+1}\,{j_s+1})}}')")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-thd_{i_s+1}_{j_s+1}*sympy.Matrix([[-(a_y-r_{i_s+1}_{j_s+1}_y)],[a_x-r_{i_s+1}_{j_s+1}_x]])'
                angular_velocity += f'-thd_{i_s+1}_{j_s+1}*sympy.Matrix([[1]])'
            else:
                linear_velocity += f'+thd_{i_s+1}_{j_s+1}*sympy.Matrix([[-(a_y-r_{i_s+1}_{j_s+1}_y)],[a_x-r_{i_s+1}_{j_s+1}_x]])'
                angular_velocity += f'+thd_{i_s+1}_{j_s+1}*sympy.Matrix([[1]])'
        elif joint_type == 2:
            if f'{i_s}_{j_s}' not in available_variables:
                executions_list.append(f"phi_{i_s+1}_{j_s+1} = sympy.symbols(r'\phi_{{({i_s+1}\,{j_s+1})}}')")
                executions_list.append(f"n_{i_s+1}_{j_s+1} = sympy.Matrix([[sympy.cos(phi_{i_s+1}_{j_s+1})],[sympy.sin(phi_{i_s+1}_{j_s+1})]])")
                executions_list.append(f"dd_{i_s+1}_{j_s+1} = sympy.symbols(r'\dot{{d}}_{{({i_s+1}\,{j_s+1})}}')")
                available_variables.append(f'{i_s}_{j_s}')
            if direction == 'descending':
                linear_velocity += f'-dd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
                angular_velocity += f'-zero_1D'
            else:
                linear_velocity += f'+dd_{i_s+1}_{j_s+1}*n_{i_s+1}_{j_s+1}'
                angular_velocity += f'+zero_1D'

    return linear_velocity, angular_velocity, executions_list, available_variables

def vel_path(M, path, available_variables):
    """
    vel_path function processes the strings to be executed for symbolic variabes and velocity expressions of a given path by using the robot-topology matrix, and it also updates the available_variables (so that only those variables that are unavailable are added, thereby avoiding redundancy). this is for spatial manipulators.

    this function takes robot-topology matrix, the path and the available variables list (string format) as input arguments.

    :param M: robot-topology matrix.
    :param path: a given path (containing link numbers in order) in list format.
    :param available_variables: a list of available variables (each of them being in string format).
    :return: the expression for linear velocity (in string format, to be executed), the expression for angular velocity (in string format, to be executed), a list of expressions for symbolic variables (in string format, to be executed), and the updated available_variables.
    """
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
                executions_list.append(f"alpha_{i_s+1}_{j_s+1} = sympy.atan(-1/(sympy.tan(beta_{i_s+1}_{j_s+1})*sympy.cos(delta_{i_s+1}_{j_s+1}-phi_{i_s+1}_{j_s+1})))")
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
    """
    get_jointvelocities_list function gets the expressions for active and passive joint velocities (in string form, to be executed), from the corresponding robot-topology matrix.

    this function takes robot-topology matrix as the input argument.

    :param M: robot-topology matrix.
    :return: a list of active joint velocities (each in string format) and a list of passive joint velocities (each in string format).
    """
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
    """
    from_P_to_P_tilde function gives P_tilde and P_tilde_omega from P, by using robot-topology matrix. it also gives the corresponding indices of P for each of the elements of P_tilde and P_tilde_omega.

    this function takes the list P and the robot-topology matrix as the input arguments.

    :param M: robot-topology matrix.
    :param P: list of all paths, denoted by P.
    :return: P_tilde in list format, P_tilde_omega in list format, independent_path_indices in list format and independent_omega_path_indices in list format.
    """
    V_i_j_vector = []
    Cv = numpy.reshape(numpy.array([]),(0,0))
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
                Cv = numpy.c_[Cv,numpy.r_[numpy.zeros((Cv.shape[0]-1,1)),numpy.array([[sortflag]])]]
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


def get_variables_list_planar(M):
    """
    get_variables_list_planar function gives the expressions for parameters (representing locations and orientations of joints, in string form), from the corresponding robot-topology matrix. this is for planar manipulators.

    this function takes robot-topology matrix as the input argument.

    :param M: robot-topology matrix.
    :return: a dict of keys representing the indices of joints and the corresponding values being lists of parameters (each parameter in string format).
    """
    variables_dict = {}
    for i in range(len(M)):
        for j in range(i+1,len(M)):
            current_variables_list = []
            if M[i,j] == 0:
                pass
            elif M[i,j] == 1:
                current_variables_list.append(f'r_{i+1}_{j+1}_x')
                current_variables_list.append(f'r_{i+1}_{j+1}_y')
            elif M[i,j] == 2:
                current_variables_list.append(f'phi_{i+1}_{j+1}')

            if len(current_variables_list) > 0:
                variables_dict[(i,j)] = current_variables_list
    return variables_dict


def get_variables_list(M):
    """
    get_variables_list function gives the expressions for parameters (representing locations and orientations of joints, in string form), from the corresponding robot-topology matrix. this is for spatial manipulators.

    this function takes robot-topology matrix as the input argument.

    :param M: robot-topology matrix.
    :return: a dict of keys representing the indices of joints and the corresponding values being lists of parameters (each parameter in string format).
    """
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
