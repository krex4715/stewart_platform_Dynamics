function [F_Joint,tau_Cartesian,L_Acc] = get_Dynamics(X,X_1,X_2,Euler,Euler_1,Euler_2)

    %%%%%%%%%% Input %%%%%%%%%%%%%%

    Platform_disp_X = [X(1),X(2),0.39 + X(3)]'; % Default Position (Height : 0.39 m)
    Platform_vel_X_1 = [X_1(1),X_1(2),X_1(3)]';
    Platform_acc_X_2 = [X_2(1),X_2(2),X_2(3)]';

    pie = deg2rad(Euler(1));
    theta = deg2rad(Euler(2)+0.01); % For avoiding Singularity
    sig = deg2rad(Euler(3));
    
    pie_1 = deg2rad(Euler_1(1));
    theta_1 = deg2rad(Euler_1(2));
    sig_1 =  deg2rad(Euler_1(3));
    
    
    pie_2 = deg2rad(Euler_2(1));
    theta_2 = deg2rad(Euler_2(2));
    sig_2 =  deg2rad(Euler_2(3));
    
 
    
    %%%%%%%%%% Platform Shape %%%%%%%%
    Vector_a1p = 0.095*[cos(deg2rad(40)), sin(deg2rad(40)),0]'; % Platform Frame
    Vector_a2p = 0.095*[cos(deg2rad(80)), sin(deg2rad(80)),0]';
    Vector_a3p = 0.095*[cos(deg2rad(160)), sin(deg2rad(160)),0]';
    Vector_a4p = 0.095*[cos(deg2rad(200)), sin(deg2rad(200)),0]';
    Vector_a5p = 0.095*[cos(deg2rad(280)), sin(deg2rad(280)),0]';
    Vector_a6p = 0.095*[cos(deg2rad(320)), sin(deg2rad(320)),0]';
    
    Vector_b1 = 0.120*[cos(deg2rad(20)), sin(deg2rad(20)),0]'; % Reference Frame
    Vector_b2 = 0.120*[cos(deg2rad(100)), sin(deg2rad(100)),0]';
    Vector_b3 = 0.120*[cos(deg2rad(140)), sin(deg2rad(140)),0]';
    Vector_b4 = 0.120*[cos(deg2rad(220)), sin(deg2rad(220)),0]';
    Vector_b5 = 0.120*[cos(deg2rad(260)), sin(deg2rad(260)),0]';
    Vector_b6 = 0.120*[cos(deg2rad(340)), sin(deg2rad(340)),0]';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    rx = [cos(sig)*cos(pie)-cos(theta)*sin(pie)*sin(sig) , cos(sig)*sin(pie)+cos(theta)*cos(pie)*sin(sig) , sin(sig)*sin(theta)]';
    ry = [-sin(sig)*cos(pie)-cos(theta)*sin(pie)*cos(sig), -sin(sig)*sin(pie)+cos(theta)*cos(pie)*cos(sig), cos(sig)*sin(theta)]';
    rz = [sin(theta)*sin(pie) , -sin(theta)*cos(pie) , cos(theta)]';
    R_wp = [rx,ry,rz];


    w_1 = [0,0,1]';
    w_2 = [cos(pie),sin(pie),0]';
    w_3 = [sin(pie)*cos(theta), -cos(pie)*sin(theta), cos(theta)]';
    Ang_vel_w = [w_1,w_2,w_3]*[pie_1,theta_1,sig_1]';




    a_1 = [0 0 0]';
    a_2 = [-pie_1*sin(pie) , pie_1*cos(pie),0]';
    a_3 = [pie_1*cos(pie)*cos(theta)-theta_1*sin(pie)*sin(theta) , pie_1*sin(pie)*sin(theta)-theta_1*cos(pie)*cos(theta) , -theta_1*sin(theta)]';
    Angular_acc_Alp = [w_1,w_2,w_3]*[pie_2,theta_2,sig_2]' + [a_1,a_2,a_3]*[pie_1,theta_1,sig_1]';




    %%%%%%%%%%%%%%%%%%%%%%Inverse Kinematics%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%c
    point_a = Platform_disp_X+R_wp*[Vector_a1p,Vector_a2p,Vector_a3p,Vector_a4p,Vector_a5p,Vector_a6p];
    
    Vector_of_Linki_L1 = point_a(:,1) - Vector_b1;
    Vector_of_Linki_L2 = point_a(:,2) - Vector_b2;
    Vector_of_Linki_L3 = point_a(:,3) - Vector_b3;
    Vector_of_Linki_L4 = point_a(:,4) - Vector_b4;
    Vector_of_Linki_L5 = point_a(:,5) - Vector_b5;
    Vector_of_Linki_L6 = point_a(:,6) - Vector_b6;
    
    scalar_L1 = sqrt(Vector_of_Linki_L1' * Vector_of_Linki_L1);
    scalar_L2 = sqrt(Vector_of_Linki_L2' * Vector_of_Linki_L2);
    scalar_L3 = sqrt(Vector_of_Linki_L3' * Vector_of_Linki_L3);
    scalar_L4 = sqrt(Vector_of_Linki_L4' * Vector_of_Linki_L4);
    scalar_L5 = sqrt(Vector_of_Linki_L5' * Vector_of_Linki_L5);
    scalar_L6 = sqrt(Vector_of_Linki_L6' * Vector_of_Linki_L6);
    
    Unit_Vector_prismatic_joint_n1 = Vector_of_Linki_L1/scalar_L1;
    Unit_Vector_prismatic_joint_n2 = Vector_of_Linki_L2/scalar_L2;
    Unit_Vector_prismatic_joint_n3 = Vector_of_Linki_L3/scalar_L3;
    Unit_Vector_prismatic_joint_n4 = Vector_of_Linki_L4/scalar_L4;
    Unit_Vector_prismatic_joint_n5 = Vector_of_Linki_L5/scalar_L5;
    Unit_Vector_prismatic_joint_n6 = Vector_of_Linki_L6/scalar_L6;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%Inverse rate Kinematics%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Velocity_of_Point_a1 = Platform_vel_X_1 + cross(Ang_vel_w,R_wp*Vector_a1p);
    Velocity_of_Point_a2 = Platform_vel_X_1 + cross(Ang_vel_w,R_wp*Vector_a2p);
    Velocity_of_Point_a3 = Platform_vel_X_1 + cross(Ang_vel_w,R_wp*Vector_a3p);
    Velocity_of_Point_a4 = Platform_vel_X_1 + cross(Ang_vel_w,R_wp*Vector_a4p);
    Velocity_of_Point_a5 = Platform_vel_X_1 + cross(Ang_vel_w,R_wp*Vector_a5p);
    Velocity_of_Point_a6 = Platform_vel_X_1 + cross(Ang_vel_w,R_wp*Vector_a6p);
    
%     Platform_vel_X_1
    Extension_Rate_of_linki_li_1 = dot(Velocity_of_Point_a1,Unit_Vector_prismatic_joint_n1);
    Extension_Rate_of_linki_li_2 = dot(Velocity_of_Point_a2,Unit_Vector_prismatic_joint_n2);
    Extension_Rate_of_linki_li_3 = dot(Velocity_of_Point_a3,Unit_Vector_prismatic_joint_n3);
    Extension_Rate_of_linki_li_4 = dot(Velocity_of_Point_a4,Unit_Vector_prismatic_joint_n4);
    Extension_Rate_of_linki_li_5 = dot(Velocity_of_Point_a5,Unit_Vector_prismatic_joint_n5);
    Extension_Rate_of_linki_li_6 = dot(Velocity_of_Point_a6,Unit_Vector_prismatic_joint_n6);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Jacobian %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    unit = [[Unit_Vector_prismatic_joint_n1]';
           [Unit_Vector_prismatic_joint_n2]';
           [Unit_Vector_prismatic_joint_n3]';
           [Unit_Vector_prismatic_joint_n4]';
           [Unit_Vector_prismatic_joint_n5]';
           [Unit_Vector_prismatic_joint_n6]'];
    
    
    j2 = [[cross(R_wp*Vector_a1p,Unit_Vector_prismatic_joint_n1)]'
           [cross(R_wp*Vector_a2p,Unit_Vector_prismatic_joint_n2)]'
           [cross(R_wp*Vector_a3p,Unit_Vector_prismatic_joint_n3)]'
           [cross(R_wp*Vector_a4p,Unit_Vector_prismatic_joint_n4)]'
           [cross(R_wp*Vector_a5p,Unit_Vector_prismatic_joint_n5)]'
           [cross(R_wp*Vector_a6p,Unit_Vector_prismatic_joint_n6)]'];
    
    jacobian_1_inv = [unit,j2];
    
    
    
    Rj = [[0,0,1]',[cos(pie),sin(pie),0]',[sin(pie)*sin(theta),-cos(pie)*sin(theta),cos(theta)]'];
    
    jacobian_2_inv = [[eye(3),zeros(3,3)];
                      [zeros(3,3),Rj]];
    
    
    Jacobian_inv = jacobian_1_inv*jacobian_2_inv;
    Jacobian = inv(Jacobian_inv);
    
    
    Jacobian_inv*[Platform_vel_X_1',Ang_vel_w']';
    %%%%%%%%%%%%%%%%%%%%%%%%%% Inverse Acceleration Kinematics %%%%%%%%%%%%%%%%%
    Acc_of_point_a1 = Platform_acc_X_2 + cross(Angular_acc_Alp,R_wp*Vector_a1p) + cross(Ang_vel_w,cross(Ang_vel_w,R_wp*Vector_a1p));
    Acc_of_point_a2 = Platform_acc_X_2 + cross(Angular_acc_Alp,R_wp*Vector_a2p) + cross(Ang_vel_w,cross(Ang_vel_w,R_wp*Vector_a2p));
    Acc_of_point_a3 = Platform_acc_X_2 + cross(Angular_acc_Alp,R_wp*Vector_a3p) + cross(Ang_vel_w,cross(Ang_vel_w,R_wp*Vector_a3p));
    Acc_of_point_a4 = Platform_acc_X_2 + cross(Angular_acc_Alp,R_wp*Vector_a4p) + cross(Ang_vel_w,cross(Ang_vel_w,R_wp*Vector_a4p));
    Acc_of_point_a5 = Platform_acc_X_2 + cross(Angular_acc_Alp,R_wp*Vector_a5p) + cross(Ang_vel_w,cross(Ang_vel_w,R_wp*Vector_a5p));
    Acc_of_point_a6 = Platform_acc_X_2 + cross(Angular_acc_Alp,R_wp*Vector_a6p) + cross(Ang_vel_w,cross(Ang_vel_w,R_wp*Vector_a6p));
    
    
    Acc_linki_li_1 = dot(Acc_of_point_a1,unit(1,:)') + dot(Velocity_of_Point_a1,(Vector_of_Linki_L1 - Extension_Rate_of_linki_li_1*unit(1,:)')/scalar_L1);
    Acc_linki_li_2 = dot(Acc_of_point_a2,unit(2,:)') + dot(Velocity_of_Point_a2,(Vector_of_Linki_L2 - Extension_Rate_of_linki_li_2*unit(2,:)')/scalar_L2);
    Acc_linki_li_3 = dot(Acc_of_point_a3,unit(3,:)') + dot(Velocity_of_Point_a3,(Vector_of_Linki_L3 - Extension_Rate_of_linki_li_3*unit(3,:)')/scalar_L3);
    Acc_linki_li_4 = dot(Acc_of_point_a4,unit(4,:)') + dot(Velocity_of_Point_a4,(Vector_of_Linki_L4 - Extension_Rate_of_linki_li_4*unit(4,:)')/scalar_L4);
    Acc_linki_li_5 = dot(Acc_of_point_a5,unit(5,:)') + dot(Velocity_of_Point_a5,(Vector_of_Linki_L5 - Extension_Rate_of_linki_li_5*unit(5,:)')/scalar_L5);
    Acc_linki_li_6 = dot(Acc_of_point_a6,unit(6,:)') + dot(Velocity_of_Point_a6,(Vector_of_Linki_L6 - Extension_Rate_of_linki_li_6*unit(6,:)')/scalar_L6);
    

    L_Acc = [Acc_linki_li_1,Acc_linki_li_2,Acc_linki_li_3,Acc_linki_li_4,Acc_linki_li_5,Acc_linki_li_6]';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ang_vel_of_link1_W1 = cross(Unit_Vector_prismatic_joint_n1,Velocity_of_Point_a1)/scalar_L1;
    ang_vel_of_link1_W2 = cross(Unit_Vector_prismatic_joint_n2,Velocity_of_Point_a2)/scalar_L2;
    ang_vel_of_link1_W3 = cross(Unit_Vector_prismatic_joint_n3,Velocity_of_Point_a3)/scalar_L3;
    ang_vel_of_link1_W4 = cross(Unit_Vector_prismatic_joint_n4,Velocity_of_Point_a4)/scalar_L4;
    ang_vel_of_link1_W5 = cross(Unit_Vector_prismatic_joint_n5,Velocity_of_Point_a5)/scalar_L5;
    ang_vel_of_link1_W6 = cross(Unit_Vector_prismatic_joint_n6,Velocity_of_Point_a6)/scalar_L6;
    
    
    
    
    ang_Acc_of_link1_Alp1 = (cross(Unit_Vector_prismatic_joint_n1,Acc_of_point_a1)-2*Extension_Rate_of_linki_li_1*ang_vel_of_link1_W1)/scalar_L1;
    ang_Acc_of_link1_Alp2 = (cross(Unit_Vector_prismatic_joint_n2,Acc_of_point_a2)-2*Extension_Rate_of_linki_li_2*ang_vel_of_link1_W2)/scalar_L2;
    ang_Acc_of_link1_Alp3 = (cross(Unit_Vector_prismatic_joint_n3,Acc_of_point_a3)-2*Extension_Rate_of_linki_li_3*ang_vel_of_link1_W3)/scalar_L3;
    ang_Acc_of_link1_Alp4 = (cross(Unit_Vector_prismatic_joint_n4,Acc_of_point_a4)-2*Extension_Rate_of_linki_li_4*ang_vel_of_link1_W4)/scalar_L4;
    ang_Acc_of_link1_Alp5 = (cross(Unit_Vector_prismatic_joint_n5,Acc_of_point_a5)-2*Extension_Rate_of_linki_li_5*ang_vel_of_link1_W5)/scalar_L5;
    ang_Acc_of_link1_Alp6 = (cross(Unit_Vector_prismatic_joint_n6,Acc_of_point_a6)-2*Extension_Rate_of_linki_li_6*ang_vel_of_link1_W6)/scalar_L6;
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    % 필요한 Parameter : m1(윗부분 무게), a11~a16 , G , Unitvector n , C
    m2 =  2.991; % 단위: kg
    m1 = 0.553;
    mp = 8.08;
    
    
    l2 = 0.1658; % 단위 m
    l1 = 0.1601; 
    
    
    
    a1_1 = cross( (scalar_L1 - l1)*ang_vel_of_link1_W1 , cross(ang_vel_of_link1_W1',unit(1,:)));
    a1_2 = cross( (scalar_L1 - l1)*ang_Acc_of_link1_Alp1 , unit(1,:));
    a1_3 = cross( 2*ang_vel_of_link1_W1 , Extension_Rate_of_linki_li_1*unit(1,:)) + Acc_linki_li_1*unit(1,:);
    
    
    a2_1 = cross( (scalar_L2 - l1)*ang_vel_of_link1_W2 , cross(ang_vel_of_link1_W2',unit(2,:)));
    a2_2 = cross( (scalar_L2 - l1)*ang_Acc_of_link1_Alp2 , unit(2,:));
    a2_3 = cross( 2*ang_vel_of_link1_W2 , Extension_Rate_of_linki_li_2*unit(2,:)) + Acc_linki_li_2*unit(2,:);
    
    a3_1 = cross( (scalar_L3 - l1)*ang_vel_of_link1_W3 , cross(ang_vel_of_link1_W3',unit(3,:)));
    a3_2 = cross( (scalar_L3 - l1)*ang_Acc_of_link1_Alp3 , unit(3,:));
    a3_3 = cross( 2*ang_vel_of_link1_W3 , Extension_Rate_of_linki_li_3*unit(3,:)) + Acc_linki_li_3*unit(3,:);
    
    a4_1 = cross( (scalar_L4 - l1)*ang_vel_of_link1_W4 , cross(ang_vel_of_link1_W4',unit(4,:)));
    a4_2 = cross( (scalar_L4 - l1)*ang_Acc_of_link1_Alp4 , unit(4,:));
    a4_3 = cross( 2*ang_vel_of_link1_W4 , Extension_Rate_of_linki_li_4*unit(4,:)) + Acc_linki_li_4*unit(4,:);
    
    a5_1 = cross( (scalar_L5 - l1)*ang_vel_of_link1_W5 , cross(ang_vel_of_link1_W5',unit(5,:)));
    a5_2 = cross( (scalar_L5 - l1)*ang_Acc_of_link1_Alp5 , unit(5,:));
    a5_3 = cross( 2*ang_vel_of_link1_W5 , Extension_Rate_of_linki_li_5*unit(5,:)) + Acc_linki_li_5*unit(5,:);
    
    a6_1 = cross( (scalar_L6 - l1)*ang_vel_of_link1_W6 , cross(ang_vel_of_link1_W6',unit(6,:)));
    a6_2 = cross( (scalar_L6 - l1)*ang_Acc_of_link1_Alp6 , unit(6,:));
    a6_3 = cross( 2*ang_vel_of_link1_W6 , Extension_Rate_of_linki_li_6*unit(6,:)) + Acc_linki_li_6*unit(6,:);
    
    
    
    
    %%% Acceleration of the center of mass of part 1
    a11 =  a1_1+a1_2+a1_3;
    a21 =  a2_1+a2_2+a2_3;
    a31 =  a3_1+a3_2+a3_3;
    a41 =  a4_1+a4_2+a4_3;
    a51 =  a5_1+a5_2+a5_3;
    a61 =  a6_1+a6_2+a6_3;
    
    %%% Acceleration of the center of mass of part 2
    a12 = cross( l2*ang_vel_of_link1_W1, cross(ang_vel_of_link1_W1',unit(1,:))) + cross( l2*ang_Acc_of_link1_Alp1, unit(1,:));
    a22 = cross( l2*ang_vel_of_link1_W2, cross(ang_vel_of_link1_W2',unit(2,:))) + cross( l2*ang_Acc_of_link1_Alp2, unit(2,:));
    a32 = cross( l2*ang_vel_of_link1_W3, cross(ang_vel_of_link1_W3',unit(3,:))) + cross( l2*ang_Acc_of_link1_Alp3, unit(3,:));
    a42 = cross( l2*ang_vel_of_link1_W4, cross(ang_vel_of_link1_W4',unit(4,:))) + cross( l2*ang_Acc_of_link1_Alp4, unit(4,:));
    a52 = cross( l2*ang_vel_of_link1_W5, cross(ang_vel_of_link1_W5',unit(5,:))) + cross( l2*ang_Acc_of_link1_Alp5, unit(5,:));
    a62 = cross( l2*ang_vel_of_link1_W6, cross(ang_vel_of_link1_W6',unit(6,:))) + cross( l2*ang_Acc_of_link1_Alp6, unit(6,:));
    
    %%%   Ni
    g = 9.8; % m/s^s
    G = [0,0,-g]';
    
    c1 = cross([1,0,0],[0,1,0]);
    c2 = cross([0,1,0],[-1,0,0]);
    c3 = cross([0,1,0],[-1,0,0]);
    c4 = cross([0,1,0],[-1,0,0]);
    c5 = cross([0,1,0],[-1,0,0]);
    c6 = cross([1,0,0],[0,1,0]);
    
    
    I_1_x = 1.7648208*10^-4;
    I_1_y = 2.7088440*10^-3;
    I_1_z = 2.7881148*10^-3;
    
    I_2_x = 3.2057427*10^-3;
    I_2_y = 2.7092491*10^-2;
    I_2_z = 2.2567492*10^-2;
    
    
    Iaa1 = I_1_z;
    Inn1 = sqrt(I_1_x^2+I_1_y^2);
    Iaa2 = I_2_z;
    Inn2 = sqrt(I_2_x^2+I_2_y^2);
    
    Ipp = [[7.7193130*10^-2, -3.4624009*10^-7, 1.0079550*10^-6];
        [-3.4624009*10^-7, 1.1936907*10^-1, 5.5786021*10^-5];
        [1.10079550*10^-6, 5.5786021*10^-5, 7.7138054*10^-2]];
    
    % Ipp = [[1.7688343*10^-1, 0 , 1.0083440*10^-6];
    %     [0, 1.1936914*10^-1 , -2.8511196*10^-5];
    %     [1.0083440*10^-6 , -2.8511196*10^-5, 1.7682828*10^-1]];
    
    
    Ip = R_wp*Ipp*R_wp';
    
    
    % Ni_1 = calculate_Ni_1(Iaa1,Iaa2,Inn1,Inn2,alp,ni)
    % Ni_2 = calculate_Ni_2(Iaa1,Iaa2,Inn1,Inn2,wi,ni)
    
    N1_1 =calculate_Ni_1(Iaa1,Iaa2,Inn1,Inn2,ang_Acc_of_link1_Alp1,unit(1,:));
    N2_1 =calculate_Ni_1(Iaa1,Iaa2,Inn1,Inn2,ang_Acc_of_link1_Alp2,unit(2,:));
    N3_1 =calculate_Ni_1(Iaa1,Iaa2,Inn1,Inn2,ang_Acc_of_link1_Alp3,unit(3,:));
    N4_1 =calculate_Ni_1(Iaa1,Iaa2,Inn1,Inn2,ang_Acc_of_link1_Alp4,unit(4,:));
    N5_1 =calculate_Ni_1(Iaa1,Iaa2,Inn1,Inn2,ang_Acc_of_link1_Alp5,unit(5,:));
    N6_1 =calculate_Ni_1(Iaa1,Iaa2,Inn1,Inn2,ang_Acc_of_link1_Alp6,unit(6,:));
    
    N1_2 = calculate_Ni_2(Iaa1,Iaa2,Inn1,Inn2,ang_vel_of_link1_W1,unit(1,:));
    N2_2 = calculate_Ni_2(Iaa1,Iaa2,Inn1,Inn2,ang_vel_of_link1_W2,unit(2,:));
    N3_2 = calculate_Ni_2(Iaa1,Iaa2,Inn1,Inn2,ang_vel_of_link1_W3,unit(3,:));
    N4_2 = calculate_Ni_2(Iaa1,Iaa2,Inn1,Inn2,ang_vel_of_link1_W4,unit(4,:));
    N5_2 = calculate_Ni_2(Iaa1,Iaa2,Inn1,Inn2,ang_vel_of_link1_W5,unit(5,:));
    N6_2 = calculate_Ni_2(Iaa1,Iaa2,Inn1,Inn2,ang_vel_of_link1_W6,unit(6,:));
    
    
    N1 = - cross(m1*(scalar_L1 - l1)*unit(1,:),G) -cross(m2*l2*unit(1,:),G) ...
        + N1_1 - N1_2 ... 
        + cross( m1*(scalar_L1 - l1)*unit(1,:) , a11) ...
        + cross( m2*l2*unit(1,:),a12);
    
    
    N2 = - cross(m1*(scalar_L2 - l1)*unit(2,:),G) -cross(m2*l2*unit(2,:),G) ...
        + N2_1 - N2_2 ... 
        + cross( m1*(scalar_L2 - l1)*unit(2,:) , a21) ...
        + cross( m2*l2*unit(2,:),a22);
    
    
    N3 = - cross(m1*(scalar_L3 - l1)*unit(3,:),G) -cross(m2*l2*unit(3,:),G) ...
        + N3_1 - N3_2 ... 
        + cross( m1*(scalar_L3 - l1)*unit(3,:) , a31) ...
        + cross( m2*l2*unit(3,:),a32);
    
    
    N4 = - cross(m1*(scalar_L4 - l1)*unit(4,:),G) -cross(m2*l2*unit(4,:),G) ...
        + N4_1 - N4_2 ... 
        + cross( m1*(scalar_L4 - l1)*unit(4,:) , a41) ...
        + cross( m2*l2*unit(4,:),a42);
    
    
    N5 = - cross(m1*(scalar_L5 - l1)*unit(5,:),G) -cross(m2*l2*unit(5,:),G) ...
        + N5_1 - N5_2 ... 
        + cross( m1*(scalar_L5 - l1)*unit(5,:) , a51) ...
        + cross( m2*l2*unit(5,:),a52);
    
    
    N6 = - cross(m1*(scalar_L6 - l1)*unit(6,:),G) -cross(m2*l2*unit(6,:),G) ...
        + N6_1 - N6_2 ... 
        + cross( m1*(scalar_L6 - l1)*unit(6,:) , a61) ...
        + cross( m2*l2*unit(6,:),a62);
    
    
    
    %%%% mi
    
    moment_m1 = dot(N1,unit(1,:))/dot(c1,unit(1,:));
    moment_m2 = dot(N2,unit(2,:))/dot(c2,unit(2,:));
    moment_m3 = dot(N3,unit(3,:))/dot(c3,unit(3,:));
    moment_m4 = dot(N4,unit(4,:))/dot(c4,unit(4,:));
    moment_m5 = dot(N5,unit(5,:))/dot(c5,unit(5,:));
    moment_m6 = dot(N6,unit(6,:))/dot(c6,unit(6,:));
    
    
    
    
    %%%  Fni
    
    Fn1 = (cross(N1,unit(1,:)) - cross(moment_m1*c1,unit(1,:))) / scalar_L1;
    Fn2 = (cross(N2,unit(2,:)) - cross(moment_m2*c2,unit(2,:))) / scalar_L2;
    Fn3 = (cross(N3,unit(3,:)) - cross(moment_m3*c3,unit(3,:))) / scalar_L3;
    Fn4 = (cross(N4,unit(4,:)) - cross(moment_m4*c4,unit(4,:))) / scalar_L4;
    Fn5 = (cross(N5,unit(5,:)) - cross(moment_m5*c5,unit(5,:))) / scalar_L5;
    Fn6 = (cross(N6,unit(6,:)) - cross(moment_m6*c6,unit(6,:))) / scalar_L6;
    
    
    
    Fn = [Fn1;Fn2;Fn3;Fn4;Fn5;Fn6];
    
    
    
    
    
    r_bar = R_wp*[0,0,0]';
    x_2_g = Platform_acc_X_2 + cross(ang_Acc_of_link1_Alp1,cross(ang_Acc_of_link1_Alp1,r_bar));
    
    
    sum_term = cross(R_wp*Vector_a1p,Fn1) + cross(R_wp*Vector_a2p,Fn2) + cross(R_wp*Vector_a3p,Fn3) ...
        +cross(R_wp*Vector_a4p,Fn4) + cross(R_wp*Vector_a5p,Fn5) + cross(R_wp*Vector_a6p,Fn6);
    
    
    
    C_1 = [mp*G-mp*x_2_g-sum(Fn ,1)'];
    C_2 = [cross(mp*r_bar,G)-mp*cross(r_bar,x_2_g)-Ip*ang_Acc_of_link1_Alp1+cross(Ip*ang_vel_of_link1_W1,ang_vel_of_link1_W1)-sum_term'];
    
    
    C = [C_1;C_2];
    
    
    
    
    
    temp = [[dot(m1*(a11'-G),unit(1,:)')]';
        [dot(m1*(a21'-G),unit(2,:)')]';
        [dot(m1*(a31'-G),unit(3,:)')]';
        [dot(m1*(a41'-G),unit(4,:)')]';
        [dot(m1*(a51'-G),unit(5,:)')]';
        [dot(m1*(a61'-G),unit(6,:)')]'];
    
    
    
%     tau = Jacobian_inv*temp - transpose(jacobian_2_inv)*C;
    
    
    
    
    J_1_T = transpose(inv(jacobian_1_inv));
    F = temp- J_1_T*C;
    tau = transpose(Jacobian_inv)*F;     
    
    
    
    
%     F_Joint = F
    
    tau_Cartesian = (tau+[0,0,0,0,0,0]')
    
    F_Joint = transpose(Jacobian)*(tau_Cartesian)
    
    
    
    
    
    
    
    
    
    
    
    
    vector1 = F_Joint(1)*unit(1,:);
    vector2 = F_Joint(2)*unit(2,:);
    vector3 = F_Joint(3)*unit(3,:);
    vector4 = F_Joint(4)*unit(4,:);
    vector5 = F_Joint(5)*unit(5,:);
    vector6 = F_Joint(6)*unit(6,:);
    
    
    
    
    o = [0,0,0];
    t = 0:0.01:1;
    
    
    

    
    % Noramlization
    Vector_b1 = Vector_b1*1000;
    Vector_b2 = Vector_b2*1000;
    Vector_b3 = Vector_b3*1000;
    Vector_b4 = Vector_b4*1000;
    Vector_b5 = Vector_b5*1000;
    Vector_b6 = Vector_b6*1000;

    Vector_p1 = point_a(:,1)*1000;
    Vector_p2 = point_a(:,2)*1000;
    Vector_p3 = point_a(:,3)*1000;
    Vector_p4 = point_a(:,4)*1000;
    Vector_p5 = point_a(:,5)*1000;
    Vector_p6 = point_a(:,6)*1000;
    




    
    figure(1)
    r = 150;
    plot3(r*cos(t*2*pi),r*sin(t*2*pi),0*t)
    hold on

    plot3((Vector_b1(1)-Vector_b2(1))*(-t)+Vector_b1(1), (Vector_b1(2)-Vector_b2(2))*(-t)+Vector_b1(2), (Vector_b1(3)-Vector_b2(3))*(-t)+Vector_b1(3),':')
    hold on
    plot3((Vector_b2(1)-Vector_b3(1))*(-t)+Vector_b2(1), (Vector_b2(2)-Vector_b3(2))*(-t)+Vector_b2(2), (Vector_b2(3)-Vector_b3(3))*(-t)+Vector_b2(3),':')
    hold on
    plot3((Vector_b3(1)-Vector_b4(1))*(-t)+Vector_b3(1), (Vector_b3(2)-Vector_b4(2))*(-t)+Vector_b3(2), (Vector_b3(3)-Vector_b4(3))*(-t)+Vector_b3(3),':')
    hold on
    plot3((Vector_b4(1)-Vector_b5(1))*(-t)+Vector_b4(1), (Vector_b4(2)-Vector_b5(2))*(-t)+Vector_b4(2), (Vector_b4(3)-Vector_b5(3))*(-t)+Vector_b4(3),':')
    hold on
    plot3((Vector_b5(1)-Vector_b6(1))*(-t)+Vector_b5(1), (Vector_b5(2)-Vector_b6(2))*(-t)+Vector_b5(2), (Vector_b5(3)-Vector_b6(3))*(-t)+Vector_b5(3),':')
    hold on
    plot3((Vector_b1(1)-Vector_b1(1))*(-t)+Vector_b6(1), (Vector_b6(2)-Vector_b1(2))*(-t)+Vector_b6(2), (Vector_b6(3)-Vector_b1(3))*(-t)+Vector_b6(3),':')
    hold on


%     plot3((Vector_p1(1)-Vector_p2(1))*(-t)+Vector_p1(1), (Vector_p1(2)-Vector_p2(2))*(-t)+Vector_p1(2), (Vector_p1(3)-Vector_p2(3))*(-t)+Vector_p1(3),':')
%     hold on
%     plot3((Vector_p2(1)-Vector_p3(1))*(-t)+Vector_p2(1), (Vector_p2(2)-Vector_p3(2))*(-t)+Vector_p2(2), (Vector_p2(3)-Vector_p3(3))*(-t)+Vector_p2(3),':')
%     hold on
%     plot3((Vector_p3(1)-Vector_p4(1))*(-t)+Vector_p3(1), (Vector_p3(2)-Vector_p4(2))*(-t)+Vector_p3(2), (Vector_p3(3)-Vector_p4(3))*(-t)+Vector_p3(3),':')
%     hold on
%     plot3((Vector_p4(1)-Vector_p5(1))*(-t)+Vector_p4(1), (Vector_p4(2)-Vector_p5(2))*(-t)+Vector_p4(2), (Vector_p4(3)-Vector_p5(3))*(-t)+Vector_p4(3),':')
%     hold on
%     plot3((Vector_p5(1)-Vector_p6(1))*(-t)+Vector_p5(1), (Vector_p5(2)-Vector_p6(2))*(-t)+Vector_p5(2), (Vector_p5(3)-Vector_p6(3))*(-t)+Vector_p5(3),':')
%     hold on
%     plot3((Vector_p1(1)-Vector_p1(1))*(-t)+Vector_p6(1), (Vector_p6(2)-Vector_p1(2))*(-t)+Vector_p6(2), (Vector_p6(3)-Vector_p1(3))*(-t)+Vector_p6(3),':')
%     hold on





    plot3(Vector_b1(1)+vector1(1)*t, Vector_b1(2)+vector1(2)*t, Vector_b1(3)+vector1(3)*t,'--')
    hold on
    plot3(Vector_b2(1)+vector2(1)*t, Vector_b2(2)+vector2(2)*t, Vector_b2(3)+vector2(3)*t,'--')
    hold on
    plot3(Vector_b3(1)+vector3(1)*t, Vector_b3(2)+vector3(2)*t, Vector_b3(3)+vector3(3)*t,'--')
    hold on
    plot3(Vector_b4(1)+vector4(1)*t, Vector_b4(2)+vector4(2)*t, Vector_b4(3)+vector4(3)*t,'--')
    hold on
    plot3(Vector_b5(1)+vector5(1)*t, Vector_b5(2)+vector5(2)*t, Vector_b5(3)+vector5(3)*t,'--')
    hold on
    plot3(Vector_b6(1)+vector6(1)*t, Vector_b6(2)+vector6(2)*t, Vector_b6(3)+vector6(3)*t,'--')
    hold on



    
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on
    title('Stewart Platform Joint Force')
    
%     figure(2)
%     
%     plot3(vector1(1)*t, vector1(2)*t, vector1(3)*t,'r')
%     hold on
%     plot3(vector2(1)*t, vector2(2)*t, vector2(3)*t,'g')
%     hold on
%     plot3(vector3(1)*t, vector3(2)*t, vector3(3)*t,'b')
%     hold on
%     plot3(vector4(1)*t, vector4(2)*t, vector4(3)*t,'m')
%     hold on
%     plot3(vector5(1)*t, vector5(2)*t, vector5(3)*t,'b')
%     hold on
%     plot3(vector6(1)*t, vector6(2)*t, vector6(3)*t,'k')
%     hold on
%     
%     
%     legend('vector1','vector2','vector3','vector4','vector5','vector6')
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
%     grid on
    
    
    


end


