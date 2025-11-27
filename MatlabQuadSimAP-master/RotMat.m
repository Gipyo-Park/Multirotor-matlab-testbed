function rm = RotMat(phi, theta, psi, int)

% Module for transforming the p vector from one frame to another frame
% Possible transformations
% 1 - Transformation from the inertial frame (i - North, j - East, k - Towards earth center) to vehicle frame 1
% 2 - Transformation from the vehicle frame 1 to vehicle frame 2
% 3 - Transformation from the vehicle frame 2 to body frame
% 4 - Transformation from the vehicle frame (inertial frame) to body frame
% 5 - Transformation from the body frame to vehicle frame (inertial frame)
% 6 - Transformation from the derivative of (phi theta psi) to body rates (p q r)
% 7 - Transformation from the body rates (p q r) to derivative of (phi theta psi)

switch int

    case 1  %Rz                                                               % Transformation from vehicle frame (i - North, j - East, k - Towards earth center) to vehicle frame 1

        rm = [ cos(psi)     sin(psi)       0  ;
            -sin(psi)     cos(psi)       0  ;
            0             0           1 ];

    case 2  %Ry                                                               % Transformation from vehicle frame 1 to vehicle frame 2

        rm = [ cos(theta)    0       -sin(theta)  ;
            0          1            0       ;
            sin(theta)    0        cos(theta) ];

    case 3     %Rx                                                            % Transformation from vehicle frame 2 to body frame
        rm = [ 1              0             0      ;
            0            cos(phi)     sin(phi)  ;
            0           -sin(phi)     cos(phi) ];

    case 4       %inv(R)                                                          
        rm = [
            (cos(psi)*cos(theta))/(cos(psi)^2*cos(theta)^2 + cos(psi)^2*sin(theta)^2 + cos(theta)^2*sin(psi)^2 + sin(psi)^2*sin(theta)^2), ...
            (cos(theta)*sin(psi))/(cos(psi)^2*cos(theta)^2 + cos(psi)^2*sin(theta)^2 + cos(theta)^2*sin(psi)^2 + sin(psi)^2*sin(theta)^2), ...
            -sin(theta)/(cos(theta)^2 + sin(theta)^2);

            -(cos(phi)*cos(theta)^2*sin(psi) + cos(phi)*sin(psi)*sin(theta)^2 - cos(psi)*sin(phi)*sin(theta))/(cos(phi)^2*cos(psi)^2*sin(theta)^2 + cos(phi)^2*cos(theta)^2*sin(psi)^2 + cos(psi)^2*cos(theta)^2*sin(phi)^2 + cos(phi)^2*sin(psi)^2*sin(theta)^2 + cos(psi)^2*sin(phi)^2*sin(theta)^2 + cos(theta)^2*sin(phi)^2*sin(psi)^2 + sin(phi)^2*sin(psi)^2*sin(theta)^2 + cos(phi)^2*cos(psi)^2*cos(theta)^2), ...
            (sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi)*cos(theta)^2 + cos(phi)*cos(psi)*sin(theta)^2)/(cos(phi)^2*cos(psi)^2*sin(theta)^2 + cos(phi)^2*cos(theta)^2*sin(psi)^2 + cos(psi)^2*cos(theta)^2*sin(phi)^2 + cos(phi)^2*sin(psi)^2*sin(theta)^2 + cos(psi)^2*sin(phi)^2*sin(theta)^2 + cos(theta)^2*sin(phi)^2*sin(psi)^2 + sin(phi)^2*sin(psi)^2*sin(theta)^2 + cos(phi)^2*cos(psi)^2*cos(theta)^2), ...
            (cos(theta)*sin(phi))/(cos(phi)^2*cos(theta)^2 + cos(phi)^2*sin(theta)^2 + cos(theta)^2*sin(phi)^2 + sin(phi)^2*sin(theta)^2);

            (cos(theta)^2*sin(phi)*sin(psi) + sin(phi)*sin(psi)*sin(theta)^2 + cos(phi)*cos(psi)*sin(theta))/(cos(phi)^2*cos(psi)^2*sin(theta)^2 + cos(phi)^2*cos(theta)^2*sin(psi)^2 + cos(psi)^2*cos(theta)^2*sin(phi)^2 + cos(phi)^2*sin(psi)^2*sin(theta)^2 + cos(psi)^2*sin(phi)^2*sin(theta)^2 + cos(theta)^2*sin(phi)^2*sin(psi)^2 + sin(phi)^2*sin(psi)^2*sin(theta)^2 + cos(phi)^2*cos(psi)^2*cos(theta)^2), ...
            -(cos(psi)*cos(theta)^2*sin(phi) + cos(psi)*sin(phi)*sin(theta)^2 - cos(phi)*sin(psi)*sin(theta))/(cos(phi)^2*cos(psi)^2*sin(theta)^2 + cos(phi)^2*cos(theta)^2*sin(psi)^2 + cos(psi)^2*cos(theta)^2*sin(phi)^2 + cos(phi)^2*sin(psi)^2*sin(theta)^2 + cos(psi)^2*sin(phi)^2*sin(theta)^2 + cos(theta)^2*sin(phi)^2*sin(psi)^2 + sin(phi)^2*sin(psi)^2*sin(theta)^2 + cos(phi)^2*cos(psi)^2*cos(theta)^2), ...
            (cos(phi)*cos(theta))/(cos(phi)^2*cos(theta)^2 + cos(phi)^2*sin(theta)^2 + cos(theta)^2*sin(phi)^2 + sin(phi)^2*sin(theta)^2)
            ];


    case 5     %R                                                            % Transformation from body frame to inertial frame
        rm = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
            sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
            -sin(theta),         cos(theta)*sin(phi),                            cos(theta)*cos(phi)];

    case 6     %inv(T)                                                            % Transformation from derivative of (phi theta psi) to body rates (p q r)
        rm = [ 1        0              -sin(theta)      ;
            0      cos(phi)     sin(phi)*cos(theta)  ;
            0     -sin(phi)     cos(phi)*cos(theta) ];

    case 7    %T                                                             % Transformation from body rates (p q r) to derivative of (phi theta psi)
        rm = [ 1, sin(phi)*tan(theta), cos(phi)*tan(theta);
            0, cos(phi), -sin(phi);
            0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    
    case 8    %transpose(R)                         % Transformation from inertial frame to body frame
        rm = [cos(psi)*cos(theta), sin(psi)*cos(theta), -sin(theta);
              cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), cos(theta)*sin(phi);
              cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi), cos(theta)*cos(phi)];

        % -------------------------------------------------------------
        % 각운동학 변환 개념 정리
        % -------------------------------------------------------------
        % T : 속도 변환 행렬 (Transformation matrix)
        %     - body 기준 각속도 [p; q; r]를 오일러각의 시간 미분 [phi_dot; theta_dot; psi_dot]로 변환
        %     - 자세각(phi, theta)에 따라 비선형적으로 달라짐
        %     - 회전행렬이 아니며, 실제 벡터 회전이 아님

        % R : 회전 행렬 (Rotation matrix)
        %     - 물리적인 벡터(속도, 가속도 등)를 body frame → inertial frame으로 회전시킴
        %     - 직교 행렬이며 공간상에서 실제 회전을 수행함

        % 핵심 차이:
        % - 각속도 [p; q; r] ≠ 오일러각 변화율 [phi_dot; theta_dot; psi_dot]
        % - T는 회전 속도의 표현 변환, R은 실제 벡터의 좌표계 회전

end

end

