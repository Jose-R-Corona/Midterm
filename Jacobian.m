function Jq1 = Jacobian(q,L)

    %Jacobian Numerial method

    H = Tz(L(1))*Rz(q(1))*Tx(L(2))*Rz(q(2))*Tx(L(3))*Tz(q(3))*Tz(-L(4))*Rz(q(4));% forward kinematics
    H=simplify(H);
    R = simplify(H(1:3,1:3));  % extract rotation matrix
    % diff by q1
    Td=Tz(L(1))*Rzd(q(1))*Tx(L(2))*Rz(q(2))*Tx(L(3))*Tz(q(3))*Tz(-L(4))*Rz(q(4))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column
    % diff by q2
    Td=Tz(L(1))*Rz(q(1))*Tx(L(2))*Rzd(q(2))*Tx(L(3))*Tz(q(3))*Tz(-L(4))*Rz(q(4))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column
    % diff by q3
    Td=Tz(L(1))*Rz(q(1))*Tx(L(2))*Rz(q(2))*Tx(L(3))*Tzd(q(3))*Tz(-L(4))*Rz(q(4))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column

    Td=Tz(L(1))*Rz(q(1))*Tx(L(2))*Rz(q(2))*Tx(L(3))*Tz(q(3))*Tz(-L(4))*Rzd(q(4))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column

        
    % Full Jacobian 6x4
    Jq1 = [J_1, J_2, J_3, J_4]   ;
end
