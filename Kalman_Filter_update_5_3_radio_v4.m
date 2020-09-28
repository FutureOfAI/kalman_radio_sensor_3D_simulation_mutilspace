% function [P00_z,K_z,z_update,Mu_z]=Kalman_Filter_update_5_3_radio(P00_z,R1m,R2m,R3m,R4m,H,R,R1m_h,R2m_h,R3m_h,R4m_h,k,c)
function [P00_z,K_z,z_update,Mu_z,Mu_1,Mu_2,Mu_3,Mu_4,Mu_5,Mu_12,Mu_13,Mu_14,Mu_23,Mu_24,Mu_34,Mu_123,Mu_124,Mu_134,Mu_234,Mu_1234]=Kalman_Filter_update_5_3_radio(P00_z,R1m,R2m,R3m,R4m,H,R,R1m_h,R2m_h,R3m_h,R4m_h,k,c)

zxm_z1(k) = R1m(k)-R1m_h(k)' ;
zxm_z2(k) = R2m(k)-R2m_h(k)' ;
zxm_z3(k) = R3m(k)-R3m_h(k)' ;
zxm_z4(k) = R4m(k)-R4m_h(k)' ;

Mu_z=[zxm_z1(k)
    zxm_z2(k)
    zxm_z3(k)
    zxm_z4(k)];

z_update = zeros(9,1); 
value = zeros(1,4); % decide threshold
%% Discrete - sequential operation
if k>1500 % after 15s, begin duration judgement
    for i = 1:4
        if abs(Mu_z(i,1)) < c
%                 A=(H(i,:)*P00_z*H(i,:)'+R(i,i))
            K_z = P00_z*H(i,:)'/(H(i,:)*P00_z*H(i,:)'+R(i,i));
            I=eye(9);
            P00_z=(I-K_z*H(i,:))*P00_z;
            value(1,i) = 1; % good signal
        else
            K_z = zeros(9,1);
            I=eye(9);
            P00_z=(I-K_z*H(i,:))*P00_z;
            value(1,i) = 0; % bad signal
        end
        z_update = z_update+K_z*Mu_z(i,1);
    end
    if(value(1,1) == 1 && value(1,2) == 1 && value(1,3) == 1 && value(1,4) == 1) % all signal is good 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 1;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 0 && value(1,2) == 1 && value(1,3) == 1 && value(1,4) == 1) % not Anchor1 signal 
        Mu_1 = 1;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 1 && value(1,2) == 0 && value(1,3) == 1 && value(1,4) == 1) % not Anchor2 signal 
        Mu_1 = 0;Mu_2 = 1;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 1 && value(1,2) == 1 && value(1,3) == 0 && value(1,4) == 1) % not Anchor3 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 1;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 1 && value(1,2) == 1 && value(1,3) == 1 && value(1,4) == 0) % not Anchor4 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 1;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 0 && value(1,2) == 0 && value(1,3) == 1 && value(1,4) == 1) % not Anchor1 Anchor2 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 1;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 0 && value(1,2) == 1 && value(1,3) == 0 && value(1,4) == 1) % not Anchor1 Anchor3 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 1;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 0 && value(1,2) == 1 && value(1,3) == 1 && value(1,4) == 0) % not Anchor1 Anchor4 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 1;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 1 && value(1,2) == 0 && value(1,3) == 0 && value(1,4) == 1) % not Anchor2 Anchor3 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 1;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 1 && value(1,2) == 0 && value(1,3) == 1 && value(1,4) == 0) % not Anchor2 Anchor4 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 1;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 1 && value(1,2) == 1 && value(1,3) == 0 && value(1,4) == 0) % not Anchor3 Anchor4 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 1;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;
    elseif(value(1,1) == 0 && value(1,2) == 0 && value(1,3) == 0 && value(1,4) == 1) % not Anchor1 Anchor2 Anchor3 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 1;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;       
    elseif(value(1,1) == 0 && value(1,2) == 0 && value(1,3) == 1 && value(1,4) == 0) % not Anchor1 Anchor2 Anchor4 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 1;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 0;         
    elseif(value(1,1) == 0 && value(1,2) == 1 && value(1,3) == 0 && value(1,4) == 0) % not Anchor1 Anchor3 Anchor4 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 1;Mu_234 = 0;
        Mu_1234 = 0;       
    elseif(value(1,1) == 1 && value(1,2) == 0 && value(1,3) == 0 && value(1,4) == 0) % not Anchor2 Anchor3 Anchor4 signal 
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 1;
        Mu_1234 = 0;       
    else
        Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 0;
        Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
        Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
        Mu_1234 = 1;       
    end
else %0~15s

    K_z = P00_z*H'/(H*P00_z*H'+R);
    z_update = K_z*Mu_z;
    I=eye(9);
    P00_z=(I-K_z*H)*P00_z; 
    Mu_1 = 0;Mu_2 = 0;Mu_3 = 0;Mu_4 = 0;Mu_5 = 1;
    Mu_12 = 0;Mu_13 = 0;Mu_14 = 0;Mu_23 = 0;Mu_24 = 0;Mu_34 = 0;
    Mu_123 = 0;Mu_124 = 0;Mu_134 = 0;Mu_234 = 0;
    Mu_1234 = 0;       
end
end

