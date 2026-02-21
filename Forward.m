
disp('Please enter the parameters for each joint when prompted.');
A = cell(3,1);
%Getting the information from the user
for i = 1:3
    disp(['Input parameters for joint ' num2str(i) ':']);
    
    theta_deg = input(['Enter theta' num2str(i) ' in degrees: ']);
    theta = theta_deg * pi / 180; 
    d = input(['Enter d' num2str(i) ' in meters: ']);
    a = input(['Enter a' num2str(i) ' in meters: ']);
    alpha_deg = input(['Enter alpha' num2str(i) ' in degrees: ']);
    alpha = alpha_deg * pi / 180; 
    
  % D-H convention method generel matrix
    A{i} = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),  a*cos(theta);
            sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0,           sin(alpha),             cos(alpha),             d;
            0,           0,                      0,                      1];
end

%Displaying the matrices
for i = 1:3
    disp(['Transformation matrix A' num2str(i) ':']);
    disp(A{i});
end

%Multiplying the three matrices
T = A{1} * A{2} * A{3};


disp('Overall transformation matrix T:');
disp(T);