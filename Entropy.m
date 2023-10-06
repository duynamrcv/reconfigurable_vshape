% X = [0.866 0.5; 0 1; -0.866 0.5; -0.866 -0.5; 0 -1; 0.866 -0.5; 5 5];
function Entro = Entropy(X)
Y = pdist(X);
R = squareform(Y);
Z = linkage(X);
% F2 = figure;
% dendrogram(Z)
% dist = max(Z(:,3));
h = [Z(:,3)'-0.0001];
Entro = 0;
Robots = [1:1:size(R,1)];
for d = 1:size(h,2)
    for i = Robots
        c{i} = i;
        Excepti = setdiff(Robots,i);
        for j = Excepti
            count = 0;
            for k = c{i}
                if R(j,k) <= h(d)
                    count = count+1;
                end
            end
            if count == size(c{i},2)
                c{i} = [c{i},j];
            end
        end
    end
    
    % Discard redundant clusters
    sumci = size(c{1},2);
    cip = c{1};
    Robots_1 = setdiff(Robots,1);
    for i = Robots_1
        if ~any(i == cip)
            sumci = sumci+size(c{i},2);
            cip = [cip,c{i}];
        else
            c{i} = [];
        end
    end
    
    Hd = 0;
    for i = Robots
        if ~isempty(c{i})
            Pj = size(c{i},2)/sumci;
            Hd = Hd - Pj*log2(Pj);
        end       
    end
    if d == 1
        Entro = Entro + Hd*h(d);
    else
        Entro = Entro + Hd*(h(d)-h(d-1));
    end
end
end