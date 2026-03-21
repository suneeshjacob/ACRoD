import sympy

is_serial = True

spatial_variables = ['x','y','z']
for k in range(6):
    for i in range(len(spatial_variables)):
        for j in range(i, len(spatial_variables)):
            string = f'I{k+1}{spatial_variables[i]}{spatial_variables[j]}=sympy.symbols("I_{{{k+1}{spatial_variables[i]}{spatial_variables[j]}}}")'
            exec(string)

for k in range(6):
    string2 = f'I{k+1} = sympy.Matrix([[I{k+1}xx, I{k+1}xy, I{k+1}xz],[I{k+1}xy, I{k+1}yy, I{k+1}yz],[I{k+1}xz, I{k+1}yz, I{k+1}zz]])'
    exec(string2)

L1 = 1
L2 = 1
L3 = 1
L4 = 1
L5 = 1
L6 = 1

m1 = 1
m2 = 1
m3 = 1
m4 = 1
m5 = 1
m6 = 1

g = 9.81

half = sympy.Rational(1,2)
ninety_degrees = sympy.Rational(1,2)*sympy.pi

t = sympy.symbols('t')

theta1 = sympy.Function('\\theta_{1}')(t)
theta2 = sympy.Function('\\theta_{2}')(t)
theta3 = sympy.Function('\\theta_{3}')(t)
theta4 = sympy.Function('\\theta_{4}')(t)
theta5 = sympy.Function('\\theta_{5}')(t)
theta6 = sympy.Function('\\theta_{6}')(t)

DH_table = [
    [ninety_degrees, 0, L1, theta1],
    [0, L2, 0, ninety_degrees+theta2],
    [ninety_degrees, 0, 0, theta3],
    [ninety_degrees, 0, L3+L4, 2*ninety_degrees+theta4],
    [ninety_degrees, 0, 0, ninety_degrees+theta5],
    [0, 0, L5+L6, theta6]
]

c = [
  [-0.5, 0, 0],
  [-0.5, 0, 0],
  [-0.5, 0, 0],
  [-0.5, 0, 0],
  [-0.5, 0, 0],
  [-0.5, 0, 0],
]

Trx = lambda x: sympy.Matrix([[1, 0, 0, 0], [0, sympy.cos(x), -sympy.sin(x), 0], [0, sympy.sin(x), sympy.cos(x), 0], [0, 0, 0, 1]])
Trz = lambda x: sympy.Matrix([[sympy.cos(x), -sympy.sin(x), 0, 0], [sympy.sin(x), sympy.cos(x), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
Ttx = lambda x: sympy.Matrix([[1, 0, 0, x], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
Ttz = lambda x: sympy.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, x], [0, 0, 0, 1]])

alpha = sympy.symbols(r'\alpha')
a = sympy.symbols(r'a')
d = sympy.symbols(r'd')
theta = sympy.symbols(r'\theta')

T_sym = Trz(theta)*Ttz(d)*Ttx(a)*Trx(alpha)
T = lambda x: T_sym.subs({alpha: x[0], a:x[1], d:x[2], theta:x[3]})

for i in range(len(DH_table)):
    string1 = f'T_{i}_{i+1} = T(DH_table[{i}])'
    exec(string1)
    if i!=0:
        string2 = f'T_0_{i+1} = T_0_{i}*T_{i}_{i+1}'
        exec(string2)

T_eff = T_0_6
p = T_eff[:3,-1]
R = T_eff[:3,:3]

for i in range(len(c)):
    string = f'c{i+1} = sympy.Matrix(c[{i}])'
    exec(string)

p1 = (T_0_1*sympy.Matrix([c1,1]))[:3]
p2 = (T_0_2*sympy.Matrix([c2,1]))[:3]
p3 = (T_0_3*sympy.Matrix([c3,1]))[:3]
p4 = (T_0_4*sympy.Matrix([c4,1]))[:3]
p5 = (T_0_5*sympy.Matrix([c5,1]))[:3]
p6 = (T_0_6*sympy.Matrix([c6,1]))[:3]

v1 = sympy.diff(p1, t)
v2 = sympy.diff(p2, t)
v3 = sympy.diff(p3, t)
v4 = sympy.diff(p4, t)
v5 = sympy.diff(p5, t)
v6 = sympy.diff(p6, t)

omega1 = Jomega[:,0]*sympy.diff(theta1,t)
omega2 = Jomega[:,1]*sympy.diff(theta2,t) + omega1
omega3 = Jomega[:,2]*sympy.diff(theta3,t) + omega2
omega4 = Jomega[:,3]*sympy.diff(theta4,t) + omega3
omega5 = Jomega[:,4]*sympy.diff(theta5,t) + omega4
omega6 = Jomega[:,5]*sympy.diff(theta6,t) + omega5

T1 = half*m1*v1.dot(v1) + half*omega1.T.dot(I1*omega1)
T2 = half*m2*v2.dot(v2) + half*omega2.T.dot(I2*omega2)
T3 = half*m3*v3.dot(v3) + half*omega3.T.dot(I3*omega3)
T4 = half*m4*v4.dot(v4) + half*omega4.T.dot(I4*omega4)
T5 = half*m5*v5.dot(v5) + half*omega5.T.dot(I5*omega5)
T6 = half*m6*v6.dot(v6) + half*omega6.T.dot(I6*omega6)

T = T1 + T2 + T3 + T4 + T5 + T6

h1 = p1[1,0]
h2 = p2[1,0]
h3 = p3[1,0]
h4 = p4[1,0]
h5 = p5[1,0]
h6 = p6[1,0]

V1 = m1*g*h1
V2 = m2*g*h2
V3 = m3*g*h3
V4 = m4*g*h4
V5 = m5*g*h5
V6 = m6*g*h6

L = T-V

tau1 = sympy.diff(sympy.diff(L, sympy.diff(theta1,t)), t) - sympy.diff(L, theta1)
tau2 = sympy.diff(sympy.diff(L, sympy.diff(theta2,t)), t) - sympy.diff(L, theta2)
tau3 = sympy.diff(sympy.diff(L, sympy.diff(theta3,t)), t) - sympy.diff(L, theta3)
tau4 = sympy.diff(sympy.diff(L, sympy.diff(theta4,t)), t) - sympy.diff(L, theta4)
tau5 = sympy.diff(sympy.diff(L, sympy.diff(theta5,t)), t) - sympy.diff(L, theta5)
tau6 = sympy.diff(sympy.diff(L, sympy.diff(theta6,t)), t) - sympy.diff(L, theta6)

theta1_sym = sympy.symbols(r'\theta_1')
theta2_sym = sympy.symbols(r'\theta_2')
theta3_sym = sympy.symbols(r'\theta_3')
theta4_sym = sympy.symbols(r'\theta_4')
theta5_sym = sympy.symbols(r'\theta_5')
theta6_sym = sympy.symbols(r'\theta_6')
theta1dot_sym = sympy.symbols(r'\dot{\theta}_1')
theta2dot_sym = sympy.symbols(r'\dot{\theta}_2')
theta3dot_sym = sympy.symbols(r'\dot{\theta}_3')
theta4dot_sym = sympy.symbols(r'\dot{\theta}_4')
theta5dot_sym = sympy.symbols(r'\dot{\theta}_5')
theta6dot_sym = sympy.symbols(r'\dot{\theta}_6')
theta1doubledot_sym = sympy.symbols(r'\ddot{\theta}_1')
theta2doubledot_sym = sympy.symbols(r'\ddot{\theta}_2')
theta3doubledot_sym = sympy.symbols(r'\ddot{\theta}_3')
theta4doubledot_sym = sympy.symbols(r'\ddot{\theta}_4')
theta5doubledot_sym = sympy.symbols(r'\ddot{\theta}_5')
theta6doubledot_sym = sympy.symbols(r'\ddot{\theta}_6')

symbols_dictionary = {
    theta1: theta1_sym,
    theta2: theta2_sym,
    theta3: theta3_sym,
    theta4: theta4_sym,
    theta5: theta5_sym,
    theta6: theta6_sym,
    sympy.diff(theta1, t): theta1dot_sym,
    sympy.diff(theta2, t): theta2dot_sym,
    sympy.diff(theta3, t): theta3dot_sym,
    sympy.diff(theta4, t): theta4dot_sym,
    sympy.diff(theta5, t): theta5dot_sym,
    sympy.diff(theta6, t): theta6dot_sym,
    sympy.diff(theta1, t,2): theta1doubledot_sym,
    sympy.diff(theta2, t,2): theta2doubledot_sym,
    sympy.diff(theta3, t,2): theta3doubledot_sym,
    sympy.diff(theta4, t,2): theta4doubledot_sym,
    sympy.diff(theta5, t,2): theta5doubledot_sym,
    sympy.diff(theta6, t,2): theta6doubledot_sym,
}

tau = sympy.Matrix([tau1, tau2, tau3, tau4, tau5, tau6])

tau_reduced = tau.subs(symbols_dictionary)

dimensional_parameters = {
    I1xx: 1,
    I1yy: 1,
    I1zz: 1,
    I1xy: 0,
    I1xz: 0,
    I1yz: 0,
    I2xx: 1,
    I2yy: 1,
    I2zz: 1,
    I2xy: 0,
    I2xz: 0,
    I2yz: 0,
    I3xx: 1,
    I3yy: 1,
    I3zz: 1,
    I3xy: 0,
    I3xz: 0,
    I3yz: 0,
    I4xx: 1,
    I4yy: 1,
    I4zz: 1,
    I4xy: 0,
    I4xz: 0,
    I4yz: 0,
    I5xx: 1,
    I5yy: 1,
    I5zz: 1,
    I5xy: 0,
    I5xz: 0,
    I5yz: 0,
    I6xx: 1,
    I6yy: 1,
    I6zz: 1,
    I6xy: 0,
    I6xz: 0,
    I6yz: 0,
    m1: 1,
    m2: 1,
    m3: 1,
    m4: 1,
    m5: 1,
    m6: 1,
    c1x: -0.5,
    c1y: 0,
    c1z: 0,
    c2x: -0.5,
    c2y: 0,
    c2z: 0,
    c3x: -0.5,
    c3y: 0,
    c3z: 0,
    c4x: -0.5,
    c4y: 0,
    c4z: 0,
    c5x: -0.5,
    c5y: 0,
    c5z: 0,
    c6x: -0.5,
    c6y: 0,
    c6z: 0,
    L1: 1,
    L2: 1,
    L3: 1,
    L4: 1,
    L5: 1,
    L6: 1,
    L7: 1,
    g: 9.81
}

tau_further_reduced = tau_reduced.subs(dimensional_parameters)

theta_and_its_derivative_symbols_vector = sympy.Matrix(list(symbols_dictionary.values()))

theta_and_its_derivative_symbols_vector

tau_vec_func = sympy.lambdify([theta_and_its_derivative_symbols_vector], tau_further_reduced)

tau_vec_func(np.random.random(size=18))
