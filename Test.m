clc
clear

syms a b c real;

rotx_a = [sym(1), sym(0), sym(0)
        sym(0), cos(a), -sin(a)
        sym(0), sin(a), cos(a)]
    
roty_b = [cos(b), sym(0), sin(b)
        sym(0), sym(1), sym(0)
        -sin(b), sym(0), cos(b)]      
 
rotz_c = [cos(c), -sin(c), sym(0)
        sin(c), cos(c), sym(0)
        sym(0), sym(0), sym(1)]

R = rotz_c * roty_b * rotx_a


b = sym(-pi/2);
simplify(eval(R))

b = sym(pi/2);
simplify(eval(R))
