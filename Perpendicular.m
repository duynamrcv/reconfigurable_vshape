function p = Perpendicular(x, a, b)
%PERPENDICULAR
d_ab = norm(a-b);
d_ax = norm(a-x);
d_bx = norm(b-x);

if d_ab ~= 0 
    if dot(a-b,x-b)*dot(b-a,x-a)>=0 % x is between a and b
        px = b(1)-a(1); py = b(2)-a(2); dAB = px*px + py*py;
        u = ((x(1) - a(1)) * px + (x(2) - a(2)) * py) / dAB;
        p = [a(1) + u * px, a(2) + u * py];
    else
        if d_ax < d_bx
            p = a;
        else
            p = b;
        end
    end
else % if a and b are identical
    p = a;
end
end

