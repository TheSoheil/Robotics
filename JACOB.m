function [Jv,Jw,H] = JACOB(DHT,Q)
% this program is developed by Soheil Hekmat
% in this function we use passed DH table(DHT) and joint variables (Q)
% to calculate jacobian matrix for each robot

% 1st section calculating H matrix: we define a 4*4 identity matrix as H
% then we use a while loop to get our desired H based on DH table we have
% we now that our table has 4 columns and we find out about rows by size()
% func. normally we have 3 rows specifically in this problem
sz = size(DHT);
H = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
rows = 1;
while sz(1,1) >= rows
    H = H * Rot('Z',DHT(rows,1));
    H = H * Trans('Z',DHT(rows,2));
    H = H * Trans('X',DHT(rows,3));
    H = H * Rot('X',DHT(rows,4));
    rows = rows + 1;
end
% 2nd section calculating Jv: we use analytical approach. paying attention 
% to v's jacobian matrix in lecture 11 we use 2 for loops to itirate
% between rows ans columns of our matrix. we need position vector which is
% in H matrix and we differentiate it by joint variables in each itiration
columns = length(Q);
syms Jv [3,columns]
for i=1:3
    for k=1:columns
        Jv(i,k) = diff(H(i,4) , Q(k,1));
    end
end
% 3rd section calculating Jw: we use analytical approach. we use 2 for
% loops and 3 if functions in the second loop to get omega's jacobian.
% respect to it's matrix in lecture 11 we use itiratation and diff to
% reach the jacobian matrix
syms Jw [3,columns]
for i=1:3
    for k=1:columns
        Jw(i,k) = 0;
        if i == 1
            Jw(i,k) = Jw(i,k) + diff(H(3,1) , Q(k,1)) * H(2,1);
            Jw(i,k) = Jw(i,k) + diff(H(3,2) , Q(k,1)) * H(2,2);
            Jw(i,k) = Jw(i,k) + diff(H(3,3) , Q(k,1)) * H(2,3);
        end
        if i == 2
            Jw(i,k) = Jw(i,k) + diff(H(1,1) , Q(k,1)) * H(3,1);
            Jw(i,k) = Jw(i,k) + diff(H(1,2) , Q(k,1)) * H(3,2);
            Jw(i,k) = Jw(i,k) + diff(H(1,3) , Q(k,1)) * H(3,3);
        end
        if i == 3
            Jw(i,k) = Jw(i,k) + diff(H(2,1) , Q(k,1)) * H(1,1);
            Jw(i,k) = Jw(i,k) + diff(H(2,2) , Q(k,1)) * H(1,2);
            Jw(i,k) = Jw(i,k) + diff(H(2,3) , Q(k,1)) * H(1,3);
        end
    end
end
end