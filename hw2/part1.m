function part1(question)
    disp(' '); disp('**********************');
    fprintf('** BEGIN QUESTION %d **\n', question);
    disp('**********************'); disp(' ');
    switch question
        case 1
            q1();
        case 2
            q2();
        case 3
            q3();
        case 4
            q4();
        case 5
            q5();
        case 6
            q6();
        otherwise
            disp('unknown question');
    end
    disp(' '); disp('********************');
    fprintf('** END QUESTION %d **\n', question);
    disp('********************'); disp(' ');
end

function q1
    % input
    R12 = [1,0,0;0,1/2,-sqrt(3)/2;0,sqrt(3)/2,1/2];
    R13 = [0,0,-1;0,1,0;1,0,0];
    
    % logic
    R21 = inv(R12);
    R23 = R21*R13;
    
    % output
    disp('R23:');
    disp(R23);
end

function q2
    % input
    m = [0.5,0,0.866;0,1,0;-0.866,0,0.5];
    
    % logic
    yRotation = {'Ctheta','0','Stheta';'0','1','0';'-Stheta','0','Ctheta'};
    theta = acosd(m(1,1));
    
    % output
    disp('Axis of rotation is Y, Y-coord wont change for any pts.');
    disp('Y-rotation matrix is:');
    disp(yRotation);
    fprintf('So theta is inverse cosine of 0.5: %0.1f degrees\n', -theta);
end

function q3
    % input
    m = [0,0,1,2;0,1,0,4;-1,0,0,5;0,0,0,1];

    % output
    disp(m);
    disp('translate by (2,4,5)');
    disp('then X become -Z, Y stays, Z becomes X');
end

function q4
    % input
    x = [0,1,0]; % this is the answer
    m = [x(1),0,-1,5;x(2),0,0,3;x(3),-1,0,2;0,0,0,1];

    % output
    disp('the rotation component must contain orthogonal vectors,');
    disp('and resulting coordinate frame must follow right hand rule:');
    disp(m);
    disp(' Y becomes -Z, Z becomes -X, so X must become Y.');
end

function q5
    % input
    m = [0.354, -0.674, 0.649, 4.33;...
         0.505, 0.722, 0.475, 2.5 ;...
         -0.788, 0.160, 0.595, 8   ;...
         0    , 0    , 0    , 1   ];

    % logic
    translation = m(1:3,4); % row 1-3 in col 4

    % ZYZ euler angles
    phi = atan2(m(2,3), m(1,3));
    psi = atan2(-m(1,1)*sin(phi) + m(2,1)*cos(phi),...
                -m(1,2)*sin(phi) + m(2,2)*cos(phi));
    theta = atan2(m(1,3)*cos(phi) + m(2,3)*sin(phi), m(3,3));

    % equiv axis rotation and direction
    eqPhi = acos((trace(m(1:3,1:3)) - 1)/2);
    U = (1/(2*sin(eqPhi)))*[m(3,2)-m(2,3);m(1,3)-m(3,1);m(2,1)-m(1,2)];
    
    % output
    disp('Translation:');
    disp(translation);
    disp('Euler angles:');
    fprintf('phi   = %0.3f rad\n', phi);
    fprintf('psi   = %0.3f rad\n', psi);
    fprintf('theta = %0.3f rad\n\n', theta);
    fprintf('equivalent to %0.3f rad rotation about axis:\n', eqPhi);
    disp(U);
end

function q6
    tTable = [1, 0, 0, 0;...
              0, 1, 0, 1;...
              0, 0, 1, 1;...
              0, 0, 0, 1];

    tCube = [1, 0, 0, -0.5;...
             0, 1, 0,  1.5;...
             0, 0, 1,  1.1;...
             0, 0, 0,  1  ];

    tCamera = [0, 1,  0, -0.5;...
               1, 0,  0,  1.5;...
               0, 0, -1,  3  ;...
               0, 0,  0,  1  ];

    tCubeCamera = tCamera\tCube;

    % output
    disp('table in base:');
    disp(tTable);
    disp('cube in base:');
    disp(tCube);
    disp('camera in base:');
    disp(tCamera);
    disp('camera in cube:');
    disp(tCubeCamera);
end
