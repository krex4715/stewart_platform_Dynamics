function Ni_1 = calculate_Ni_1(Iaa1,Iaa2,Inn1,Inn2,alp,ni)
    a1 = (Iaa1+Iaa2)*(dot(alp,ni))*ni;
    a2 = cross((Inn1+Inn2)*ni,cross(alp,ni));

    Ni_1 = a1+a2;
end