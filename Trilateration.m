function [N1 N2] = Trilateration(P,S,W)
[mp,np] = size(P);
ns = length(S);
if (ns~=np)
    error('Number of reference points and distances are different');
end
A=[]; b=[];
for i1=1:np
    x = P(1,i1); y = P(2,i1); z = P(3,i1);
    s = S(i1);
    A = [A ; 1 -2*x  -2*y  -2*z]; 
    b= [b ; s^2-x^2-y^2-z^2 ];
end
if (np==3)
    warning off;
    Xp= A\b;  % Gaussian elimination

    xp = Xp(2:4,:);
    Z = null(A,'r');
    z = Z(2:4,:);
    if rank (A)==3
        %Polynom coeff.
        a2 = z(1)^2 + z(2)^2 + z(3)^2 ;
        a1 = 2*(z(1)*xp(1) + z(2)*xp(2) + z(3)*xp(3))-Z(1);
        a0 = xp(1)^2 +  xp(2)^2+  xp(3)^2-Xp(1);
        p = [a2 a1 a0];
        t = roots(p);

        %Solutions
        N1 = Xp + t(1)*Z;
        N2 = Xp + t(2)*Z;
    end
end
if  (np>3)
%Particular solution

    if W~=diag(ones(1,length(W)))
        C = W'*W;
        Xpdw =inv(A'*C*A)*A'*C*b; % Solution with Weights Matrix
    else
        Xpdw=pinv(A)*b; % Solution without Weights Matrix
    end

    N1 = Xpdw;
    N2 = N1;
end