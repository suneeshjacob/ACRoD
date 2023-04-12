import sympy
import numpy
import itertools

def unique_list(input_list):
    unique = []
    for i in range(len(input_list)):
        if input_list[i] not in unique:
            unique.append(input_list[i])
    return unique

def jointslist(A):
    n=len(A)
    temp1=[]
    temp2=[]
    temp3=[]
    temp4=[]
    temp5=[]
    for i in range(n):
        for j in range(i+1,n):
            if A[i,j]==1:
                temp1.append((i,j,))
            elif A[i,j]==2:
                temp2.append((i,j,))
            elif A[i,j]==3:
                temp3.append((i,j,))
            elif A[i,j]==4:
                temp4.append((i,j,))
            elif A[i,j]==5:
                temp5.append((i,j,))
#            temp.append(A[i,j])
#    return [temp.count(1),temp.count(2),temp.count(3),temp.count(4)]
    return [len(temp1),len(temp2),len(temp3),len(temp4),len(temp5)]

def iscircular(list1,list2):
    if len(list1)!=len(list2):
        return False
    temp1=' '.join(map(str,list1))
    temp2=' '.join(map(str,list2))
    if len(temp1)!=len(temp2):
        return False
    temp3=temp1+' '+temp1
    if temp2 in temp3:
        return True
    else:
        return False

def reducedcyclic(valueslist):
    for i in range(len(valueslist)):
        if valueslist[i] in valueslist[i+1:]:
            return valueslist[i:i+valueslist[i+1:].index(valueslist[i])+1]

def listloops(A):
    n=len(A)
    a=[]
    for i in range(n):
        temp=[]
        for j in list(range(0,i))+list(range(i+1,n)):
            k=[j,i]
            k.sort()
            if A[k[0],k[1]]!=0:
                temp.append(j)
        a.append(temp)
    
#    temp=[]
    temp5=[]
    for i in range(n):
        temp=[]
        flag=0
        temp.append([i])
        if len(a[i])==0:
            flag=1
            ls=[]
        else:
            temp=[temp[0]+[i2] for i2 in a[i]]
        if flag==0:
            temp2=[]
            for i3 in temp:
                for i4 in a[i3[-1]]:
                    if i4!=i3[-2]:
                        temp2.append(i3+[i4])
        flag2=0
        while flag2==0:
            temp2=[]
            for i3 in temp:
                for i4 in a[i3[-1]]:
                    if i4!=i3[-2]:
                        temp2.append(i3+[i4])
            temp=temp2
            flag2=1
            for i4 in temp:
                if len(set(i4))==len(i4):
                    flag2=0
                    break
        if len(temp)!=0:
            temp6=[]
            for i5 in temp:
                temp8=reducedcyclic(i5)
                if len(temp8)==len(set(temp8)):
                    temp6.append(temp8)
            temp=temp6
            temp5+=temp
    if len(temp5)!=0:
        temp7=[temp5[0]]
        for i in range(len(temp5)):
            for j in range(len(temp7)):
                flag3=0
                if iscircular(temp5[i],temp7[j])==True:
                    flag3=1
                    break
                elif iscircular(temp5[i][::-1],temp7[j])==True:
                    flag3=1
                    break
            if flag3==0:
                temp7.append(temp5[i])
    else:
        temp7=temp5
    return temp7

def getalljoints(A,linknumber):
    n=len(A)
    k=list(range(0,linknumber))+list(range(linknumber+1,n))
    kvtemp=[]
    for itemp in k:
        ktemp=[linknumber,itemp]
        ktemp.sort()
        kvtemp.append(int(A[ktemp[0],ktemp[1]]))
    return kvtemp

def getalllinks(A,linknumber):
    k=getalljoints(A,linknumber)
    k2=k[:linknumber]+[9]+k[linknumber:]
    k3=[i for i in range(len(k2)) if ((k2[i]!=0)&(k2[i]!=9))]
    return k3

def getconnections(A):
    chains=[[A.shape[0]-1]]
    while(sum([1 if ii[-1]!=0 else 0 for ii in chains])>0):
        tempchains=[]
        for i in chains:
            templinklist=getalllinks(A,i[-1])
            templinklist=[j for j in templinklist if j not in i[:-1]]
            for j in templinklist:
                if i[-1]!=0:
                    tempchains.append(i+[j])
        tempchains=tempchains+[i for i in chains if i[-1]==0]
        chains=tempchains
    return chains

def finalconnections(A):
    normalconnections=getconnections(A)
    loops=listloops(A)
    loopconnections=[]
    def rotate(l,n,direction='l'):
        if direction=='l':
            return l[n:]+l[:n]
        elif direction=='r':
            return l[:n]+l[n:]
        else:
            return None
    for i in loops:
        if (0 in i)&(len(A)-1 in i):
            for j in range(len(i)):
                temprotatelist=rotate(i,j+1)
                if temprotatelist[0]==len(A)-1:
                    if temprotatelist[-1]==0:
                        loopconnections.append(i)
                        break
    """
    for i in normalconnections:
        for j in loopconnections:
            if iscircular(i,j)==False:
                totalconnections.append(i)
    """
    totalconnections=[]
    totalconnections=loopconnections
    for i in normalconnections:
        if i not in totalconnections:
            totalconnections.append(i)
    return totalconnections

def issphericalcontrollable(A_mat,superfluous = False, baseend = True, firstreturn = True):
    superfluous_dof = 0
    if len(A_mat)%2 == 0:
        range_var = range(1,int((len(A_mat)/2)+1))
    else:
        #range_var = range(1,int(((len(A_mat)-1)/2)+1+1))
        range_var = range(1,int(((len(A_mat)-1)/2)+1))

    done_list = []
    found_list = []
    superfluous_data_list = []
    uncontrollable_list = []
    
    superfluous_links_data_list = []
        
    for i in range_var:
        combs = itertools.combinations(range(len(A_mat)),i)
        for j in combs:
            combs2_single = sorted(list(set(range(len(A_mat)))-set(j)))
            if list(sorted(j)) not in done_list and list(sorted(combs2_single)) not in done_list:
                done_list.append(list(sorted(j)))
                done_list.append(list(sorted(combs2_single)))
                meshgrid_var = numpy.meshgrid(j,combs2_single)
                #coupling_content_var = [[eval(f'A_mat{sorted([meshgrid_var[0][j3,j2],meshgrid_var[1][j3,j2]])}') for j3 in range(len(combs2_single))] for j2 in range(len(j))]
                A_mat_ivals = [[sorted([meshgrid_var[0][j3,j2],meshgrid_var[1][j3,j2]]) for j3 in range(len(combs2_single))] for j2 in range(len(j))]
                coupling_content_var = [[A_mat[j3[0],j3[1]] for j3 in j2] for j2 in A_mat_ivals]
                coupling_matrix_var = numpy.array(coupling_content_var)
                sum1 = sum(sum(coupling_matrix_var==4))
                if sum1 == 2:
                    sum2 = sum(sum(coupling_matrix_var==1)) + sum(sum(coupling_matrix_var==2)) + sum(sum(coupling_matrix_var==3)) + sum(sum(coupling_matrix_var==4)) + sum(sum(coupling_matrix_var==5))
                    if sum1==sum2:
                        if {0,len(A_mat)-1}<=set(j) or {0,len(A_mat)-1}<=set(combs2_single):
                            superfluous_links_list = []
                            superfluous_data_list.append([j,combs2_single])
                            for j2 in range(len(coupling_content_var)):
                                for j3 in range(len(coupling_content_var[j2])):
                                    if coupling_content_var[j2][j3]==4:
                                        superfluous_links_list.append(A_mat_ivals[j2][j3])
                            non_base_end_part = j if 0 not in j else combs2_single
                            superfluous_links = [[j5 for j5 in j4 if j5 in non_base_end_part][0] for j4 in superfluous_links_list]
                            superfluous_links_data_list.append([[j,combs2_single],[[set(superfluous_links_list[j4]),superfluous_links[j4]] for j4 in range(len(superfluous_links_list))]])
                        else:
                            uncontrollable_list.append([j,combs2_single])
                            if firstreturn == True:
                                return [False,[j,combs2_single]]
                        # if baseend == True:
                        #     #parts_indices_var = numpy.where(coupling_matrix_var==1)
                        #     #numpy.array(j)[parts_indices_var[0]],numpy.array(combs2_single)[parts_indices_var[1]]
                        #     if {0,len(A_mat)-1}<set(j) or {0,len(A_mat)-1}<set(combs2_single):
                        #         superfluous_dof += 1
                        #         superfluous_data_list.append([j,combs2_single])
                        #     else:
                        #         if superfluous == True:
                        #             if firstreturn == True:
                        #                 return [False,[j,combs2_single]]
                        #             else:
                        #                 found_list.append([j,combs2_single])
                        #         else:
                        #             if firstreturn == True:
                        #                 return False
                        #             else:
                        #                 return False
                        # else:
                        #     superfluous_dof += 1
            else:
                pass

    
    if firstreturn == False:
        #return [superfluous_data_list, uncontrollable_list]
        return [superfluous_links_data_list, uncontrollable_list]
    else:
        return [True, None]

    
def velosforpath(A,i_current, symbolicvariableslist = []):
    
    angulist_temporary = []
    velolist_temporary = []
    latexstringtemp = []
    latexomegastringtemp = []
    tobeexecuted = []
    for j in range(len(i_current)-1):
        latextemp=[]
        latexomegatemp=[]

        i_current_pair_temp = [i_current[j],i_current[j+1]]
        i_current_pair_temp2 = [str(i8+1) for i8 in i_current_pair_temp]
        i_current_pair = sorted(i_current_pair_temp)
        current_joint_temp = A[i_current_pair[0],i_current_pair[1]]
        current_index_temp = f'_{i_current_pair_temp[0]+1}_{i_current_pair_temp[1]+1}'
        current_index_reverse_temp = f'_{i_current_pair_temp[1]+1}_{i_current_pair_temp[0]+1}'
        current_index = f'({i_current_pair_temp[0]+1}\,{i_current_pair_temp[1]+1})'
        current_index_reverse = f'({i_current_pair_temp[1]+1}\,{i_current_pair_temp[0]+1})'
        if current_joint_temp == 1:
            if f'thd{current_index_reverse_temp}' in symbolicvariableslist:
                velolist_temporary.append(f'(-thd{current_index_reverse_temp}*n{current_index_temp}.cross(a-r{current_index_temp}))[:2,:]')
                latextemp.append(r'-\dot{\theta}_{'+current_index_reverse+r'} \hat{n}_{'+current_index+r'} \times \left( \vec{a} - \vec{r}_{'+current_index+r'} \right)')
                angulist_temporary.append(f'(-thd{current_index_reverse_temp}*n{current_index_temp})[2,:]')
                latexomegatemp.append(r'-\dot{\theta}_{'+current_index_reverse+r'} \hat{n}_{'+current_index+r'}')
            else:
                for j2 in ['x','y']:
                    tobeexecuted.append(f'r{current_index_temp}{j2}=sympy.symbols(\'r_{{{current_index}{j2}}}\')')
                    symbolicvariableslist.append(f'r{current_index_temp}{j2}')
                tobeexecuted.append(f'n{current_index_temp}=sympy.Matrix([0,0,1])')
                tobeexecuted.append(f'r{current_index_temp}=sympy.Matrix([r{current_index_temp}x,r{current_index_temp}y,0])')
                tobeexecuted.append(f'thd{current_index_temp}=sympy.symbols(\'\\dot{{\\\\theta}}_{{{current_index}}}\')')
                symbolicvariableslist.append(f'thd{current_index_temp}')
                velolist_temporary.append(f'(thd{current_index_temp}*n{current_index_temp}.cross(a-r{current_index_temp}))[:2,:]')
                latextemp.append(r'\dot{\theta}_{'+current_index+r'} \hat{k}_{'+current_index+r'} \times \left( \vec{a} - \vec{r}_{'+current_index+r'} \right)')
                angulist_temporary.append(f'(thd{current_index_temp}*n{current_index_temp})[2,:]')
                latexomegatemp.append(r'\dot{\theta}_{'+current_index+r'} \hat{k}_{'+current_index+r'}')
        elif current_joint_temp == 2:
            if f'dd{current_index_reverse_temp}' in symbolicvariableslist:
                velolist_temporary.append(f'(-dd{current_index_reverse_temp}*n{current_index_temp})[:2,:]')
                latextemp.append(r'-\dot{d}_{'+current_index_reverse+r'} \hat{n}_{'+current_index+r'}')
            else:
                for j2 in ['x','y']:
                    tobeexecuted.append(f'n{current_index_temp}{j2}=sympy.symbols(\'n_{{{current_index}{j2}}}\')')
                    symbolicvariableslist.append(f'n{current_index_temp}{j2}')
                    tobeexecuted.append(f'r{current_index_temp}{j2}=sympy.symbols(\'r_{{{current_index}{j2}}}\')')
                    symbolicvariableslist.append(f'r{current_index_temp}{j2}')
                tobeexecuted.append(f'n{current_index_temp}=sympy.Matrix([n{current_index_temp}x,n{current_index_temp}y,0])')
                tobeexecuted.append(f'r{current_index_temp}=sympy.Matrix([r{current_index_temp}x,r{current_index_temp}y,0])')
                tobeexecuted.append(f'dd{current_index_temp}=sympy.symbols(\'\\dot{{d}}_{{{current_index}}}\')')
                symbolicvariableslist.append(f'dd{current_index_temp}')
                velolist_temporary.append(f'(dd{current_index_temp}*n{current_index_temp})[:2,:]')
                latextemp.append(r'\dot{d}_{'+current_index+r'} \hat{n}_{'+current_index+r'}')

        
        latexstringtemp.append('+'.join([i3 for i3 in latextemp if len(i3)>0]))
        latexomegastringtemp.append('+'.join([i3 for i3 in latexomegatemp if len(i3)>0]))
    #velo = sum(velolist_temporary,sympy.zeros(3,1))
    latexstring = ' + '.join([i3 for i3 in latexstringtemp if len(i3)>0])
    #angu = sum(angulist_temporary,sympy.zeros(3,1))
    latexomegastring = ' + '.join([i3 for i3 in latexomegastringtemp if len(i3)>0])

    return [tobeexecuted,velolist_temporary,angulist_temporary,latexstring,latexomegastring,symbolicvariableslist]


def splitmatrixintotwomatrices(A_new):
    old_matrix = numpy.matrix([[A_new[i,j] if i<=j else 0 for j in range(A_new[i].shape[1])] for i in range(A_new.shape[0])])
    actuation_matrix = numpy.matrix([[A_new[i,j] if i>j else 0 for j in range(A_new[i].shape[1])] for i in range(A_new.shape[0])],dtype=int)
    return [old_matrix, actuation_matrix]

def jacobian(A_new, robottype='planar'):
    
    splittedmatrices = splitmatrixintotwomatrices(A_new)
    old_matrix = splittedmatrices[0]
    actuation_matrix = splittedmatrices[1]
    
    
    
    A = old_matrix
    
    actuation_info = list(sorted(zip(*numpy.where(actuation_matrix == 1))))
    
    symbolicvariablesstrlist = []
    
    ax=sympy.symbols('a_x')
    symbolicvariablesstrlist.append('ax')
    ay=sympy.symbols('a_y')
    symbolicvariablesstrlist.append('ay')

    if robottype == 'planar':
        a=sympy.Matrix([ax,ay,0])
    else:
        az=sympy.symbols('a_z')
        symbolicvariablesstrlist.append('az')
        a=sympy.Matrix([ax,ay,az])


    reserved_superfluous = set({})
    
    latexstring=[]
    latexomegastring=[]
    velolist=[]
    angulist=[]
    
    finalconn_old=finalconnections(A)

    Y = []
    #Y_sortflag_list = []
    Z = numpy.reshape(numpy.matrix(''),(0,0))
    for i in range(len(finalconn_old)):
        Z = numpy.r_[Z,numpy.zeros((1,len(Y)))]
        for j in range(len(finalconn_old[i])-1):
            k_unsorted = [finalconn_old[i][j],finalconn_old[i][j+1]]
            k = list(sorted(k_unsorted))
            if k == k_unsorted:
                sortflag_temp = 1
            elif k == k_unsorted[::-1]:
                sortflag_temp = -1
            if k not in Y:
                Y.append(k)
                Z = numpy.c_[Z,numpy.r_[numpy.zeros((Z.shape[0]-1,1)),numpy.matrix(sortflag_temp)]]
            elif k in Y:
                ind = Y.index(k)
                Z[i,ind]= sortflag_temp

    mr = numpy.linalg.matrix_rank(Z)
    mr_flag = 0
    comb_generatortemp = itertools.combinations(range(Z.shape[0]),mr)
    while mr_flag == 0:
        comb_temp = next(comb_generatortemp)
        if numpy.linalg.matrix_rank(Z[comb_temp,:]) == mr:
            comb_final = comb_temp
            mr_flag = 1


    booleans_for_Z_columns = [False if A[i[0],i[1]]==2 else True for i in Y]
    Z2 = Z[comb_final,:][:,booleans_for_Z_columns]
    mr2 = numpy.linalg.matrix_rank(Z2)
    mr_flag2 = 0
    comb_generatortemp2 = itertools.combinations(range(Z2.shape[0]),mr2)
    while mr_flag2 == 0:
        comb_temp2 = next(comb_generatortemp2)
        if numpy.linalg.matrix_rank(Z2[comb_temp2,:]) == mr2:
            comb_final2 = comb_temp2
            mr_flag2 = 1


    finalconn = [finalconn_old[i] for i in range(len(finalconn_old)) if i in comb_final]
    finalconn2 = [finalconn[i] for i in range(len(finalconn)) if i in comb_final2]

    angulist_flags = []

    for i in finalconn:
        if i[0]==len(A)-1:
            i_current = i[::-1]
        tobeexecuted,velolist_temporary,angulist_temporary,latexvelo,latexangu,symbolicvariablesstrlist_temp = velosforpath(A,i_current,symbolicvariableslist=symbolicvariablesstrlist)
        for j in tobeexecuted:
            exec(j)
        #print(locals().keys())
        #eval('thd_1_2')
        velolist_temp_matrix = sympy.zeros(2,1)
        for j in velolist_temporary:
            velolist_temp_matrix += eval(j)
        velolist.append(velolist_temp_matrix)
        #velolist.append(sum([eval(j) for j in velolist_temporary],sympy.zeros(3,1)))
        latexstring.append(latexvelo)
        if i in finalconn2:
            angulist_temp_matrix = sympy.zeros(1,1)
            for j in angulist_temporary:
                angulist_temp_matrix += eval(j)
            angulist.append(angulist_temp_matrix)
            angulist_flags.append(True)
            #angulist.append(sum([eval(j) for j in angulist_temporary],sympy.zeros(3,1)))
        else:
            angulist.append(sympy.Matrix([sympy.nan for i in range(1)]))
            angulist_flags.append(False)
        latexomegastring.append(latexangu)
        symbolicvariablesstrlist = symbolicvariablesstrlist_temp
    velo = velolist[-1]
    angu = angulist[-1]  

    
    #symbolicvariablesstrlist = [str(j) for j in symbolicvariableslist]
    symbolicvariableslist = []
    for j in symbolicvariablesstrlist:
        symbolicvariableslist.append(eval(j))
    #symbolicvariableslist = [eval(j) for j in symbolicvariablesstrlist]
    kinds_of_actuation = [numpy.matrix(A,dtype=int)[i[1],i[0]] for i in actuation_info]
    active_velocities = set({})
    for i in range(len(actuation_info)):
        if kinds_of_actuation[i]==1:
            #actuator_string = '\\dot{\\theta}_{('+str(actuation_info[i][0]+1)+','+str(actuation_info[i][1]+1)+')}'
            actuator_string = 'thd_'+str(actuation_info[i][0]+1)+'_'+str(actuation_info[i][1]+1)+''
            if actuator_string in symbolicvariablesstrlist:
                active_velocities.add(symbolicvariableslist[symbolicvariablesstrlist.index(actuator_string)])
            else:
                #actuator_string = '\\dot{\\theta}_{('+str(actuation_info[i][1]+1)+','+str(actuation_info[i][0]+1)+')}'
                actuator_string = 'thd_'+str(actuation_info[i][1]+1)+'_'+str(actuation_info[i][0]+1)+''
                active_velocities.add(symbolicvariableslist[symbolicvariablesstrlist.index(actuator_string)])
        elif kinds_of_actuation[i]==2:
            #actuator_string = '\\dot{d}_{('+str(actuation_info[i][0]+1)+','+str(actuation_info[i][1]+1)+')}'
            actuator_string = 'dd_'+str(actuation_info[i][0]+1)+'_'+str(actuation_info[i][1]+1)+''
            if actuator_string in symbolicvariablesstrlist:
                active_velocities.add(symbolicvariableslist[symbolicvariablesstrlist.index(actuator_string)])
            else:
                #actuator_string = '\\dot{d}_{('+str(actuation_info[i][1]+1)+','+str(actuation_info[i][0]+1)+')}'
                actuator_string = 'dd_'+str(actuation_info[i][1]+1)+'_'+str(actuation_info[i][0]+1)+''
                active_velocities.add(symbolicvariableslist[symbolicvariablesstrlist.index(actuator_string)])

    full_symbols = set({})
    for i in velolist+angulist:
        for j in i.free_symbols:
            full_symbols.add(j)
    full_velocities = {i for i in full_symbols if str(i)[0] not in ['a','n','r']}

    passive_velocities = full_velocities - active_velocities
    
    active_velocities_sorted = sorted(active_velocities,key=lambda i:str(i))
    passive_velocities_sorted = sorted(passive_velocities,key=lambda i:str(i))
    

    
    superfluousflag=0
    superfluouscontent=None
    superfluouslink=None
    angucons=[]
    





    superfluous_equations = []
    issphericalcontrollable_info = issphericalcontrollable(A, firstreturn = False)
    if len(issphericalcontrollable_info[1])==0:
        for i in range(len(issphericalcontrollable_info[0])):
            superfluous_point1_indices_string = '_'.join(list(map(lambda x:str(x+1),issphericalcontrollable_info[0][i][1][0][0]))[::-1])
            if 'r_'+superfluous_point1_indices_string+'x' not in symbolicvariablesstrlist:
                superfluous_point1_indices_string = '_'.join(list(map(lambda x:str(x+1),issphericalcontrollable_info[0][i][1][0][0])))
            superfluous_point1 = ['r_'+superfluous_point1_indices_string+j+'' for j in ['x','y','z']]
            superfluouspoint1function = sympy.Matrix([symbolicvariableslist[symbolicvariablesstrlist.index(superfluous_point1[j])] for j in range(3)])
            
            superfluous_point2_indices_string = '_'.join(list(map(lambda x:str(x+1),issphericalcontrollable_info[0][i][1][1][0]))[::-1])
            if 'r_'+superfluous_point2_indices_string+'x' not in symbolicvariablesstrlist:
                superfluous_point2_indices_string = '_'.join(list(map(lambda x:str(x+1),issphericalcontrollable_info[0][i][1][1][0])))
            superfluous_point2 = ['r_'+superfluous_point2_indices_string+j+'' for j in ['x','y','z']]
            superfluouspoint2function = sympy.Matrix([symbolicvariableslist[symbolicvariablesstrlist.index(superfluous_point2[j])] for j in range(3)])
            
            #print(superfluous_point1,superfluous_point2)

            
            
            
            a_superfluous_link = issphericalcontrollable_info[0][i][1][0][1]
            path_involving_superfluous_link = [j for j in finalconn if a_superfluous_link in j][0]
            if path_involving_superfluous_link[0]==0:
                path_containing_superfluous_link = path_involving_superfluous_link
            else:
                path_containing_superfluous_link = path_involving_superfluous_link[::-1]
            path_to_superfluous_link = path_containing_superfluous_link[:path_containing_superfluous_link.index(a_superfluous_link)+1]
            
            absolute_velocity_of_a_superfluous_link_str = velosforpath(A,path_to_superfluous_link)[1]
            superfluous_velocity_direction = superfluouspoint2function-superfluouspoint1function
            absolute_velocity_of_a_superfluous_link = sympy.zeros(3,1)
            for j in absolute_velocity_of_a_superfluous_link_str:
                absolute_velocity_of_a_superfluous_link += eval(j)
                
            superfluous_equations.append(absolute_velocity_of_a_superfluous_link.dot(superfluous_velocity_direction))
            
            
    else:
        print('Robot uncontrollable')




    J_temp = sympy.Matrix([velolist[0],angulist[0]])
    J1 = J_temp.jacobian(active_velocities_sorted)
    if len(passive_velocities_sorted)>0:
        J2 = J_temp.jacobian(passive_velocities_sorted)

        A_temp = sympy.Matrix([])
        for i in range(1,len(velolist)):
            #A_temp = A_temp.row_insert(0,sympy.Matrix([velolist[i]-velolist[0],angulist[i]-angulist[0]]))
            A_temp = A_temp.row_insert(0,sympy.Matrix([velolist[i]-velolist[0]]))
            if angulist_flags[i]==True:
                A_temp = A_temp.row_insert(0,sympy.Matrix([angulist[i]-angulist[0]]))

        if len(superfluous_equations)!=0:
            A_temp = A_temp.row_insert(A_temp.shape[0],sympy.Matrix(superfluous_equations))

        A1 = A_temp.jacobian(active_velocities_sorted)
        A2 = A_temp.jacobian(passive_velocities_sorted)
    else:
        J2 = None
        A1 = None
        A2 = None
        

    dictionary_of_substitutions = {}


    ninds = sorted(set([str(i)[3:-2] for i in symbolicvariableslist if str(i).startswith('n_')]))

    for i in ninds:
        tempindsstr = i[1:-1].split(',')
        exec(f'delta_{tempindsstr[0]}_{tempindsstr[1]}=sympy.symbols("\\\\delta_{{({tempindsstr[0]}\\,{tempindsstr[1]})}}")')
        dictionary_of_substitutions[[j for j in symbolicvariableslist if str(j)==f'n_{{{i}x}}'][0]]=eval(f'sympy.cos(delta_{tempindsstr[0]}_{tempindsstr[1]})')
        dictionary_of_substitutions[[j for j in symbolicvariableslist if str(j)==f'n_{{{i}y}}'][0]]=eval(f'sympy.sin(delta_{tempindsstr[0]}_{tempindsstr[1]})')



    # for i in ninds:
    #     tempindsstr = i[1:-1].split(',')
    #     exec(f'beta_{tempindsstr[0]}_{tempindsstr[1]}=sympy.symbols("\\\\beta_{{({tempindsstr[0]}\\,{tempindsstr[1]})}}")')
    #     exec(f'phi_{tempindsstr[0]}_{tempindsstr[1]}=sympy.symbols("\\phi_{{({tempindsstr[0]}\\,{tempindsstr[1]})}}")')
    #     dictionary_of_substitutions[[j for j in symbolicvariableslist if str(j)==f'n_{{{i}x}}'][0]]=eval(f'sympy.sin(beta_{tempindsstr[0]}_{tempindsstr[1]})*sympy.cos(phi_{tempindsstr[0]}_{tempindsstr[1]})')
    #     dictionary_of_substitutions[[j for j in symbolicvariableslist if str(j)==f'n_{{{i}y}}'][0]]=eval(f'sympy.sin(beta_{tempindsstr[0]}_{tempindsstr[1]})*sympy.sin(phi_{tempindsstr[0]}_{tempindsstr[1]})')
    #     dictionary_of_substitutions[[j for j in symbolicvariableslist if str(j)==f'n_{{{i}z}}'][0]]=eval(f'sympy.cos(beta_{tempindsstr[0]}_{tempindsstr[1]})')


    # minds = sorted(set([str(i)[3:-2] for i in symbolicvariableslist if str(i).startswith('m_')]))
    # for i in minds:
    #     tempindsstr = i[1:-1].split(',')
    #     exec(f'alpha_{tempindsstr[0]}_{tempindsstr[1]}=sympy.symbols("\\alpha_{{({tempindsstr[0]}\\,{tempindsstr[1]})}}")')
    #     exec(f'delta_{tempindsstr[0]}_{tempindsstr[1]}=sympy.symbols("\\delta_{{({tempindsstr[0]}\\,{tempindsstr[1]})}}")')
    #     dictionary_of_substitutions[[j for j in symbolicvariableslist if str(j)==f'm_{{{i}x}}'][0]]=eval(f'sympy.sin(alpha_{tempindsstr[0]}_{tempindsstr[1]})*sympy.cos(delta_{tempindsstr[0]}_{tempindsstr[1]})')
    #     dictionary_of_substitutions[[j for j in symbolicvariableslist if str(j)==f'm_{{{i}y}}'][0]]=eval(f'sympy.sin(alpha_{tempindsstr[0]}_{tempindsstr[1]})*sympy.sin(delta_{tempindsstr[0]}_{tempindsstr[1]})')
    #     dictionary_of_substitutions[[j for j in symbolicvariableslist if str(j)==f'm_{{{i}z}}'][0]]=eval(f'sympy.cos(alpha_{tempindsstr[0]}_{tempindsstr[1]})')


    J1_new = J1.subs(dictionary_of_substitutions)
    if len(passive_velocities_sorted)>0:
        J2_new = J2.subs(dictionary_of_substitutions)
        A1_new = A1.subs(dictionary_of_substitutions)
        A2_new = A2.subs(dictionary_of_substitutions)


        new_variables = sorted(J1_new.free_symbols.union(J2_new.free_symbols).union(A1_new.free_symbols).union(A2_new.free_symbols),key=lambda x:str(x))


    else:
        J2_new = None
        A1_new = None
        A2_new = None

        new_variables = sorted(J1_new.free_symbols,key=lambda x:str(x))

    
    return [J1,J2,A1,A2,[active_velocities_sorted,passive_velocities_sorted],[latexstring,latexomegastring],symbolicvariableslist,new_variables,[J1_new,J2_new,A1_new,A2_new],0]


