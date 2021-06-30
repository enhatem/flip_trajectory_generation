% extracting number of rows and columns from phi
[r, c] = size(phi');

% defining the theta and psi angles
theta = zeros(r,c);
psi = theta;


%% euler angles 
euler = [ phi' , theta , psi ];

quat = eul2quat(euler,'XYZ');

%% normalizing quat
for i = 1:r
    qi = quat(i);
    qi_norm = sqrt(sum(qi)^2);
    quat(i) = 1 / qi_norm * qi;
end

%% q_dot

% defining q_dot
q_dot = diff(quat) / t_step;

%% angular velocities

w = zeros(r-1,4);

for i=1:r-1
    qi = quat(i,:);
    q_dot_i = q_dot(i,:);
    temp = 2 * quatmultiply(q_dot_i, quatinv(qi));
    w(i,:) = temp;
end

% removing the unwanted column
w = w(:,2:end);
wx = w(:,1);
wy = w(:,2);
wz = w(:,3);