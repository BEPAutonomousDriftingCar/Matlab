

n2 = 1;
for  n= 1:4
    [pk, locs] = find(B.Data(:,n));
    while n2 < length(pk)
%    dT = B.Time(pk(n2+1)) - B.Time(pk(n2));
     
    dT = (pk(n2+1)-pk(n2))*0.0112267;
    w = (pi)/dT;
    B.Data(pk(n2):pk(n2+1),n) = w;
    n2 = n2 + 1; 
    
    end
    n2 =1;
end

