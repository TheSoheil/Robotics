function P = Path_generator_LimitedEdition (Xs, Xf, eta, B)
% This function is developed by Soheil Hekmat respect to problem 4
% this function mainly is same as the first function but it will debug the
% problem we are facing in question 3 with our new obstacle
stop_factor = 0.1; Epsilon = 0.1; Alpha = 1;
X = Xs;
doi = 2; % doi is distance of influence given in question 2
k = 1;
while norm(X - Xf) >= stop_factor
    Fatt = -eta * (X - Xf);
    Frep = 0;
    for i = 1:length(B)
        % in this part based on lectures 19 & 20 we wanna use matrix B to
        % find Bi which is minimum distance between X and obstacle. to
        % reach this goal we will use lines and their equation. A and C are
        % start and end of the line (one side of a polygon)
        A = [B(i,1);B(i,2)];
        C = [B(i,3);B(i,4)];
        D = [(A - C),(-[0,-1;1,0] * (A - C))];
        % now we wanna multiply inverse of D by X-C to get alfa
        alfa = D \ (X - C);
        % alfa is a 2*1 matrix which we need only the first element of it
        if alfa(1,1) <= 1 && alfa(1,1) >= 0
        Bi = alfa(1,1) * (A - C) + C;
        elseif alfa(1,1) < 0
            Bi = C;
        else
            Bi = A;
        end
        ROi = norm(Bi - X);
        if ROi <= doi
            Frep = Frep + (Alpha * (1/ROi^3) * (1/ROi - 1/doi) * (X - Bi));
        else
            Frep = Frep + 0;
        end
    end
    F = Fatt + Frep;
    Fd = F / norm(F);
    X = X + Epsilon * Fd;
    P(:,k) = X;
    % here we will check if new point of the path is has the same x or y
    % which our two previous point or not, because it may cause trouble for
    % the algorithm and it may get stuck
    sz = size(P);
    if sz(1,2) > 3 && P(2,sz(1,2)) == P(2,sz(1,2) - 2) && P(1,sz(1,2)) == P(1,sz(1,2) - 2)
        while true
        jump = randi([0 6],1,1);
        theta = 0 + rand*(2*pi-0);
        X_temp = X + [jump*cos(theta); jump*sin(theta)];
        check = Collision_99106074(X,X_temp,B);
        if check == 0
            X = X + [jump*cos(theta); jump*sin(theta)];
            break
        end
        end
    end
    k = k + 1;
end
end
