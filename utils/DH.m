function T = DH( d,a, alpha, theta)
%PLANARMANIPULATOR Construct an instance of this class
%Generates homogenous transformation matrix from DH parameters!
T =[ cos(theta)  -cos(alpha)*sin(theta)  sin(alpha)*sin(theta) a*cos(theta) ;
        sin(theta)   cos(alpha)*cos(theta) -sin(alpha)*cos(theta) a*sin(theta) ;
        0            sin(alpha)             cos(alpha)            d            ;
        0            0                      0                   1           ];
    
    
end