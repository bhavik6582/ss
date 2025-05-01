"use client"

import { Button } from "@/components/ui/button"
import { useState } from "react"

export function ButtonGrid() {
  const [copied, setCopied] = useState(false)
  const [copiedCoord, setCopiedCoord] = useState(false)
  const [copied3Axis, setCopied3Axis] = useState(false)
  const [copied3D, setCopied3D] = useState(false)
  const [copied2D, setCopied2D] = useState(false)
  const [copiedTemplate, setCopiedTemplate] = useState(false)
  const [copiedScara, setCopiedScara] = useState(false)
  const [copiedYPR, setCopiedYPR] = useState(false)
  const [copiedDH, setCopiedDH] = useState(false)
  const [copiedComposite, setCopiedComposite] = useState(false)

  const handleCopy = () => {
    const matlabCode = `wpts = [0 45 15 90 45; 90 45 -45 15 90];
[q, qd, qdd, tvec, ppv] = trapveltraj(wpts, 501);

subplot(2,2,1)
plot(tvec, q)
xlabel('t')
ylabel('Position')
legend('X', 'Y')

subplot(2,1,2)
plot(tvec, qd)
xlabel('t')
ylabel('Velocities')
legend('X', 'Y')`
    
    navigator.clipboard.writeText(matlabCode)
    setCopied(true)
    setTimeout(() => setCopied(false), 2000)
  }

  const handleCopyCoord = () => {
    const matlabCode = `clear;
p = [1; 2; 3];
rotation_angle = 90;
theta = deg2rad(rotation_angle);
Rz = [cos(theta), -sin(theta), 0;
      sin(theta),  cos(theta), 0;
      0,           0,          1];
disp('Z-Axis Rotation matrix');
disp(Rz);
transformed_point = Rz * p;
disp('original point');
disp(p);
disp('Transformed point');
disp(transformed_point);`
    
    navigator.clipboard.writeText(matlabCode)
    setCopiedCoord(true)
    setTimeout(() => setCopiedCoord(false), 2000)
  }

  const handleCopy3Axis = () => {
    const matlabCode = `% 3-axis code
clc; clear; close all;

% Link lengths
a1 = 1;
a2 = 1.5;
a3 = 1;

% Joint angles (in radians)
t1 = pi/3;
t2 = pi/12;
t3 = pi/18;

% Forward kinematics calculations
a1x = a1 * cos(t1);
a1y = a1 * sin(t1);
a2x = a1x + a2 * cos(t1 + t2);
a2y = a1y + a2 * sin(t1 + t2);
a3x = a2x + a3 * cos(t1 + t2 + t3);
a3y = a2y + a3 * sin(t1 + t2 + t3);

% Coordinate axes
xx = [-5, 5];
xy = [0, 0];
yx = [0, 0];
yy = [-5, 5];

% Link positions
link1x = [0, a1x];
link1y = [0, a1y];
link2x = [a1x, a2x];
link2y = [a1y, a2y];
link3x = [a2x, a3x];
link3y = [a2y, a3y];

% Plot the robotic arm
figure;
hold on;
grid on;
axis equal;
plot(xx, xy, 'r', yx, yy, 'g'); % Plot reference axes
plot(link1x, link1y, 'b', 'LineWidth', 2); % Link 1
plot(link2x, link2y, 'c', 'LineWidth', 2); % Link 2
plot(link3x, link3y, 'm', 'LineWidth', 2); % Link 3

% Scatter joint and end-effector positions
scatter([0, a1x, a2x, a3x], [0, a1y, a2y, a3y], 50, 'filled', 'k');

% Label points
text(0, 0, ' O (0,0)', 'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold');
text(a1x, a1y, sprintf(' A (%.2f, %.2f)', a1x, a1y), 'FontSize', 10, 'Color', 'b', 'FontWeight', 'bold');
text(a2x, a2y, sprintf(' B (%.2f, %.2f)', a2x, a2y), 'FontSize', 10, 'Color', 'c', 'FontWeight', 'bold');
text(a3x, a3y, sprintf(' P (%.2f, %.2f)', a3x, a3y), 'FontSize', 10, 'Color', 'm', 'FontWeight', 'bold');

% Titles and labels
title('3-Axis Robotic Arm Forward Kinematics');
xlabel('X-axis');
ylabel('Y-axis');
hold off;`
    
    navigator.clipboard.writeText(matlabCode)
    setCopied3Axis(true)
    setTimeout(() => setCopied3Axis(false), 2000)
  }

  const handleCopy3D = () => {
    const matlabCode = `PM = [1 0 0 1]
%initialize the transformation matrix as Identity
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]
frame = input("Enter the frame you want fixed(1) or mobile(2): ")
if(frame == 1)
%First Operation
op = input("Enter your operation as 1 for Rotation or 2 for Translation: ")
if(op == 1)
angle = input("Enter the input angle for Rotation: ")
axis = input("Enter the input axis of Roatation(from 1,2 or 3): ")
if(axis == 1)
HR = [1 0 0 0 ; 0 cos(angle) -sin(angle) 0; 0 sin(angle) cos(angle) 0; 0 0 0 1]
elseif(axis == 2)
HR = [cos(angle) 0 sin(angle) 0; 0 1 0 0; -sin(angle) 0 cos(angle) 0; 0 0 0 1]
elseif(axis == 3)
HR = [cos(angle) -sin(angle) 0 0; sin(angle) cos(angle) 0 0; 0 0 1 0; 0 0 0 1]
end
H1 = HR*T
else
dist = input("Enter the distance of Translation: ")
T_axis = input("Enter the Axis on which the previous translation has happned(1,2 or 3): ")
if(T_axis == 1)
HT = [1 0 0 dist; 0 1 0 0; 0 0 1 0; 0 0 0 1]
elseif(T_axis == 2)
HT = [1 0 0 0; 0 1 0 dist; 0 0 1 0; 0 0 0 1]
elseif(T_axis == 3)
HT = [1 0 0 0; 0 1 0 0; 0 0 1 dist; 0 0 0 1]
end
H1 = HT*T
end
%Second Operation
op = input("Enter your second operation as 1 for Rotation or 2 for Translation: ")
if(op == 1)
angle = input("Enter the input angle for Rotation: ")
axis = input("Enter the input axis of Roatation(from 1,2 or 3): ")
if(axis == 1)
HR = [1 0 0 0 ; 0 cos(angle) -sin(angle) 0; 0 sin(angle) cos(angle) 0; 0 0 0 1]
elseif(axis == 2)
HR = [cos(angle) 0 sin(angle) 0; 0 1 0 0; -sin(angle) 0 cos(angle) 0; 0 0 0 1]
elseif(axis == 3)
HR = [cos(angle) -sin(angle) 0 0; sin(angle) cos(angle) 0 0; 0 0 1 0; 0 0 0 1]
end
H2 = HR*T
else
dist = input("Enter the distance of Translation: ")
T_axis = input("Enter the Axis on which the previous translation has happned(1,2 or 3): ")
if(T_axis == 1)
HT = [1 0 0 dist; 0 1 0 0; 0 0 1 0; 0 0 0 1]
elseif(T_axis == 2)
HT = [1 0 0 0; 0 1 0 dist; 0 0 1 0; 0 0 0 1]
elseif(T_axis == 3)
HT = [1 0 0 0; 0 1 0 0; 0 0 1 dist; 0 0 0 1]
end
H2 = HT*T
end
H = H2*H1
PF = H*transpose(PM)
else
%First Operation
op = input("Enter your operation as 1 for Rotation or 2 for Translation: ")
if(op == 1)
angle = input("Enter the input angle for Rotation: ")
axis = input("Enter the input axis of Roatation(from 1,2 or 3): ")
if(axis == 1)
HR = [1 0 0 0 ; 0 cos(angle) -sin(angle) 0; 0 sin(angle) cos(angle) 0; 0 0 0 1]
elseif(axis == 2)
HR = [cos(angle) 0 sin(angle) 0; 0 1 0 0; -sin(angle) 0 cos(angle) 0; 0 0 0 1]
elseif(axis == 3)
HR = [cos(angle) -sin(angle) 0 0; sin(angle) cos(angle) 0 0; 0 0 1 0; 0 0 0 1]
end
H1 = T*HR
else
dist = input("Enter the distance of Translation: ")
T_axis = input("Enter the Axis on which the previous translation has happned(1,2 or 3): ")
if(T_axis == 1)
HT = [1 0 0 dist; 0 1 0 0; 0 0 1 0; 0 0 0 1]
elseif(T_axis == 2)
HT = [1 0 0 0; 0 1 0 dist; 0 0 1 0; 0 0 0 1]
elseif(T_axis == 3)
HT = [1 0 0 0; 0 1 0 0; 0 0 1 dist; 0 0 0 1]
end
H1 = T*HT
end
%Second Operation
op = input("Enter your second operation as 1 for Rotation or 2 for Translation: ")
if(op == 1)
angle = input("Enter the input angle for Rotation: ")
axis = input("Enter the input axis of Roatation(from 1,2 or 3): ")
if(axis == 1)
HR = [1 0 0 0 ; 0 cos(angle) -sin(angle) 0; 0 sin(angle) cos(angle) 0; 0 0 0 1]
elseif(axis == 2)
HR = [cos(angle) 0 sin(angle) 0; 0 1 0 0; -sin(angle) 0 cos(angle) 0; 0 0 0 1]
elseif(axis == 3)
HR = [cos(angle) -sin(angle) 0 0; sin(angle) cos(angle) 0 0; 0 0 1 0; 0 0 0 1]
end
H2 = T*HR
else
dist = input("Enter the distance of Translation: ")
T_axis = input("Enter the Axis on which the previous translation has happned(1,2 or 3): ")
if(T_axis == 1)
HT = [1 0 0 dist; 0 1 0 0; 0 0 1 0; 0 0 0 1]
elseif(T_axis == 2)
HT = [1 0 0 0; 0 1 0 dist; 0 0 1 0; 0 0 0 1]
elseif(T_axis == 3)
HT = [1 0 0 0; 0 1 0 0; 0 0 1 dist; 0 0 0 1]
end
H2 = T*HT
end
H = H1*H2
PF = H*transpose(PM)
end`
    
    navigator.clipboard.writeText(matlabCode)
    setCopied3D(true)
    setTimeout(() => setCopied3D(false), 2000)
  }

  const handleCopy2D = () => {
    const matlabCode = `original_point = [1, 1, 1];  % idhar points if change krna karo

translation = [2, 3, 0];  % Translation according to quest

% z ka rotation angle
theta = 45;
theta = deg2rad(theta);  % Convert to radians

% Rz ka formula
Rz = [cos(theta), -sin(theta), 0;
      sin(theta),  cos(theta), 0;
      0,           0,          1];

% Apply the translation
translated_point = original_point + translation;

% Apply the rotation (first translate, then rotate)
final_point = (Rz * translated_point')';

% Display the results
disp('Original Point:');
disp(original_point);
disp('Translated Point:');
disp(translated_point);
disp('Final Point after Rotation:');
disp(final_point);`
    
    navigator.clipboard.writeText(matlabCode)
    setCopied2D(true)
    setTimeout(() => setCopied2D(false), 2000)
  }

  const handleCopyTemplate = () => {
    const matlabCode = `% Template Matching
clear; clc; close all;

% Load the image
img = imread('image.jpg');
template = imread('template.jpg');

% Convert to grayscale if needed
if size(img, 3) == 3
    img = rgb2gray(img);
end
if size(template, 3) == 3
    template = rgb2gray(template);
end

% Perform template matching
result = normxcorr2(template, img);

% Find the maximum correlation
[max_val, max_idx] = max(result(:));
[y, x] = ind2sub(size(result), max_idx);

% Adjust coordinates for the template size
x = x - size(template, 2) + 1;
y = y - size(template, 1) + 1;

% Display results
figure;
imshow(img);
hold on;
rectangle('Position', [x, y, size(template, 2), size(template, 1)], ...
         'EdgeColor', 'r', 'LineWidth', 2);
title('Template Matching Result');
hold off;`
    
    navigator.clipboard.writeText(matlabCode)
    setCopiedTemplate(true)
    setTimeout(() => setCopiedTemplate(false), 2000)
  }

  const handleCopyScara = () => {
    const matlabCode = `%% SCARA Robot Kinematics

% Given parameters (positions, dimensions)
w1 = 692.82;
w2 = 25;
w3 = 527;
w4 = 0;   % Not used in current calculations
w5 = 0;   % Not used in current calculations
w6 = 1.6487;

a1 = 425;
a2 = 375;
d1 = 877;
d4 = 200;

% Joint 2 angle (theta2 in degrees)
theta2 = 60;
theta2_rad = deg2rad(theta2);  % Convert to radians

% Joint 2 calculation using Law of Cosines (inverse kinematics)
% If using acos formula directly for angle:
q2 = acos(((w1^2 + w2^2 - a1^2 - a2^2) / (2 * a1 * a2)));

% Joint 1 calculation using atan2 for orientation
numerator = (a2 * sin(theta2_rad)) * w1 + (a1 + a2 * cos(theta2_rad)) * w2;
denominator = (a1 + a2 * cos(theta2_rad)) * w1 - a2 * sin(theta2_rad) * w2;
q1 = atan2(numerator, denominator);

% Prismatic joint displacement
q3 = d1 - d4 - w3;

% Joint 4 rotation based on natural log
q4 = pi * log(abs(w6));

% Display results
fprintf('q1 (rad): %.4f\n', q1);
fprintf('q2 (rad): %.4f\n', q2);
fprintf('q3 (mm): %.2f\n', q3);
fprintf('q4 (rad): %.4f\n', q4);`
    
    navigator.clipboard.writeText(matlabCode)
    setCopiedScara(true)
    setTimeout(() => setCopiedScara(false), 2000)
  }

  const handleCopyYPR = () => {
    const matlabCode = `axis_type = input('Select axis type: 1 for mobile, 2 for fixed: ');
frame_order = upper(input('Enter the rotation order (e.g., "XYZ", "ZYX", "YPR"): ', 's'));
angles_deg = input('Enter rotation angles in degrees as a vector [theta1, theta2, theta3]: ');
angles_rad = deg2rad(angles_deg);

Rx = @(theta) [1, 0, 0;
               0, cos(theta), -sin(theta);
               0, sin(theta), cos(theta)];
           
Ry = @(theta) [cos(theta), 0, sin(theta);
               0, 1, 0;
               -sin(theta), 0, cos(theta)];
           
Rz = @(theta) [cos(theta), -sin(theta), 0;
               sin(theta), cos(theta), 0;
               0, 0, 1];

R = eye(3);

if axis_type == 1  % Mobile axes
    for i = 1:length(frame_order)
        if frame_order(i) == 'X'
            R = Rx(angles_rad(i)) * R;
        elseif frame_order(i) == 'Y'
            R = Ry(angles_rad(i)) * R;
        elseif frame_order(i) == 'Z'
            R = Rz(angles_rad(i)) * R;
        else
            error('Invalid rotation axis specified. Use X, Y, or Z.');
        end
    end
elseif axis_type == 2  % Fixed axes
    for i = length(frame_order):-1:1
        if frame_order(i) == 'X'
            R = Rx(angles_rad(i)) * R;
        elseif frame_order(i) == 'Y'
            R = Ry(angles_rad(i)) * R;
        elseif frame_order(i) == 'Z'
            R = Rz(angles_rad(i)) * R;
        else
            error('Invalid rotation axis specified. Use X, Y, or Z.');
        end
    end
else
    error('Invalid axis type. Use 1 for mobile or 2 for fixed.');
end

disp('Composite Rotation Matrix:');
disp(R);

inp_point = input('Enter a 3D point as a vector [x, y, z]: ');
inp_point = reshape(inp_point, 3, 1);
final_point = R * inp_point;

disp('Final Point after rotation:');
disp(final_point);`
    
    navigator.clipboard.writeText(matlabCode)
    setCopiedYPR(true)
    setTimeout(() => setCopiedYPR(false), 2000)
  }

  const handleCopyDH = () => {
    const matlabCode = `clc;
clear;
close all;

n = input('Enter number of joints: ');
fprintf('Enter q (Theta in radians) as a vector: ');
q = input('');

fprintf('Enter d (link offset) as a vector: ');
d = input('');

fprintf('Enter a (link length) as a vector: ');
a = input('');

fprintf('Enter Alpha (link twist, in radians) as a vector: ');
alpha = input('');

T = eye(4);  % Initialize transformation matrix

for i = 1:n
    theta = q(i);
    d_i = d(i);
    a_i = a(i);
    alpha_i = alpha(i);

    c_i = cos(theta);
    s_i = sin(theta);
    c_alpha = cos(alpha_i);
    s_alpha = sin(alpha_i);

    A = [c_i, -s_i*c_alpha,  s_i*s_alpha, a_i*c_i;
         s_i,  c_i*c_alpha, -c_i*s_alpha, a_i*s_i;
         0,    s_alpha,      c_alpha,     d_i;
         0,    0,            0,           1];

    T = T * A;  % Multiply transformations
end

fprintf('\nFinal Transformation Matrix:\n');
disp(T);

position_vector = T(1:3, 4);
fprintf('Position Vector:\n');
disp(position_vector);`
    
    navigator.clipboard.writeText(matlabCode)
    setCopiedDH(true)
    setTimeout(() => setCopiedDH(false), 2000)
  }

  const handleCopyComposite = () => {
    const matlabCode = `% Input vectors
w = input("Enter the value for w as a vector [w1 w2 w3 w4 w5 w6]: ");
d = input("Enter the value for d as a vector [d1 d2 d3 d4 ...]: ");
a = input("Enter the value for a as a vector [a1 a2]: ");

% Calculate q2 using Law of Cosines
q2 = acos(((w(1)^2 + w(2)^2 - a(1)^2 - a(2)^2) / (2 * a(1) * a(2))));

% Forward kinematic calculations for two configurations (elbow up/down)
Y = (a(2) * sin(q2) * w(1)) + (a(1) + a(2) * cos(q2)) * w(2);
X = (a(1) + a(2) * cos(q2)) * w(1) - a(2) * sin(q2) * w(2);
q1a = atan2(Y, X);  % Elbow-down solution

Yb = (a(2) * sin(-q2) * w(1)) + (a(1) + a(2) * cos(-q2)) * w(2);
Xb = (a(1) + a(2) * cos(-q2)) * w(1) - a(2) * sin(-q2) * w(2);
q1b = atan2(Yb, Xb);  % Elbow-up solution

% Compute prismatic joint value q3
q3 = d(1) - d(4) - w(3);

% Compute q4 from w6
q4 = pi * log(abs(w(6)));

% Display results
disp("q2 = "); disp(q2);
disp("q1a = "); disp(q1a);
disp("q1b = "); disp(q1b);
disp("q3 = "); disp(q3);
disp("q4 = "); disp(q4);`
    
    navigator.clipboard.writeText(matlabCode)
    setCopiedComposite(true)
    setTimeout(() => setCopiedComposite(false), 2000)
  }

  return (
    <div className="flex flex-col gap-4 p-4 items-center">
      <Button 
        variant="default" 
        onClick={handleCopy}
        className={copied ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copied ? "Copied!" : "trajectory_planning"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopyCoord}
        className={copiedCoord ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copiedCoord ? "Copied!" : "Coordinate_Transformation_problem_YPR_RPY"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopy3Axis}
        className={copied3Axis ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copied3Axis ? "Copied!" : "3-Axis Robotic Arm Forward Kinematics"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopy3D}
        className={copied3D ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copied3D ? "Copied!" : "Composite_homogeneous(3D)"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopy2D}
        className={copied2D ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copied2D ? "Copied!" : "Composite_homogeneous(2D)"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopyTemplate}
        className={copiedTemplate ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copiedTemplate ? "Copied!" : "Template_Matching"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopyScara}
        className={copiedScara ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copiedScara ? "Copied!" : "scara_joint_variables_6"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopyYPR}
        className={copiedYPR ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copiedYPR ? "Copied!" : "YPR_RPY_equivalence_3"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopyDH}
        className={copiedDH ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copiedDH ? "Copied!" : "direct_kinematics_3_axis_DH"}
      </Button>
      <Button 
        variant="default" 
        onClick={handleCopyComposite}
        className={copiedComposite ? "bg-green-500 hover:bg-green-600" : ""}
      >
        {copiedComposite ? "Copied!" : "Composite homogeneous coordinate transformations"}
      </Button>
    </div>
  )
} 