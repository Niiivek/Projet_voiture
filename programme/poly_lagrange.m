function direction = GetDirection(xRight, yRight, xLeft, yLeft, xCar, yCar, v)
    %Regression d'ordre m sur les points de droites et de gauche.
    m = 3;
    n = size(xRight');
    n = n(1);
    V = zeros(n, m + 1);
    for i = 1: n
        for j = 1: m + 1
            V(i, j) = xRight(i) ^ (j - 1);
        end
    end
    aRight = (V' * V)\(V' * (yRight'));
    %pareil pour les points de gauches

    n = size(xLeft');
    n = n(1);
    V = zeros(n, m + 1);
    for i = 1:n
        for j = 1:m+1
            V(i, j) = xLeft(i)^(j - 1);
        end
    end
    aLeft = (V' * V)\(V' * (yLeft'));
    
    %calculé la direction a suivre!

end

