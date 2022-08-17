function Ni_2 = calculate_Ni_2(Iaa1,Iaa2,Inn1,Inn2,wi,ni)
    I = Iaa1+Iaa2-Inn1-Inn2;
    a1 = I*dot(wi,ni)*ni;
    Ni_2 = cross(a1,wi);
end