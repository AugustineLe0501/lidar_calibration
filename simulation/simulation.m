clear all
close all 
%CONFIGURATION
% Gaussian distribution
sigma = 0.01;

% Default plane

normal_vector = [0 0 1]; 
point_on_plane = [0 0 1];

normal_vector_2 = [1 0 0]; 
point_on_plane_2 = [0.7 0 0];

normal_vector_3 = [1 0 0]; 
point_on_plane_3 = [-1.2 0 0];


%lidar frame
lidar_frame = [0.002 -0.005 0.000]; %m
rpy = [deg2rad(-3) deg2rad(4) deg2rad(0)];

% Create a mesh for azimuth and elevation
azimuth = -180:1:180;
elevation = -135:1:135;


%%END CONFIGURATION
azimuth_to_rad = wrapToPi(deg2rad(azimuth));
elevation_to_rad = wrapToPi(deg2rad(elevation));

[Azi,Ele] = meshgrid(azimuth_to_rad,elevation_to_rad);

azi_ele_matrix = [Ele(:), Azi(:)];

vector_in_mount_frame = [0 0 1];
vector_lidar_frame = zeros(size(azi_ele_matrix(:,1),1), 3);
point_on_line_2 = zeros(size(azi_ele_matrix(:,1),1),3);

for i = 1:size(vector_lidar_frame(:,1),1)
    Translate_matrix = makehgtform('xrotate',rpy(1),'yrotate',rpy(2),'zrotate',(3),'translate',lidar_frame); %transform from mount to camera
    Rotation_matrix_y = makehgtform('yrotate',azi_ele_matrix(i,1));
    Rotation_matrix_z = makehgtform('zrotate',azi_ele_matrix(i,2));

    calib_matrix = Rotation_matrix_z*Translate_matrix*Rotation_matrix_y;

    temp = calib_matrix*cart2hom(vector_in_mount_frame)';
    vector_lidar_frame(i,:) = hom2cart(temp');

    vector_lidar_frame(i,:) = vector_lidar_frame(i,:) - lidar_frame;
    % find the 2nd point on the lines
    translate = makehgtform('translate',10*vector_lidar_frame(i,:));
    temp = translate*cart2hom(lidar_frame)';
    temp = temp';
    point_on_line_2(i,:) = hom2cart(temp);
end
%plane Z

intersect_point = zeros([],3);
range = zeros([],1);
check_matrix = [];
azi_ele_matrix_exclude = zeros([],2);
for i = 1:size(point_on_line_2(:,1),1)
    [I,check]=plane_line_intersect(normal_vector,point_on_plane,lidar_frame,point_on_line_2(i,:)); %Zplane
    [I1,check1]=plane_line_intersect(normal_vector_2,point_on_plane_2,lidar_frame,point_on_line_2(i,:)); %Xplane
    [I2,check2]=plane_line_intersect(normal_vector_3,point_on_plane_3,lidar_frame,point_on_line_2(i,:)); %Yplane

    if (check==1)&&(check1==1)&&(check2==1)
        range_z = norm(I-lidar_frame);
        range_x = norm(I1-lidar_frame);
        range_y = norm(I1-lidar_frame);
        [B, C]=sort([range_z range_x range_y]);
        switch C(1)
            case 1
                intersect_point(end+1,:) = I;
                range(end+1,:) = normrnd(1,sigma)*norm(I-lidar_frame);
            case 2
                intersect_point(end+1,:) = I1;
                range(end+1,:) = normrnd(1,sigma)*norm(I1-lidar_frame);
            case 3
                intersect_point(end+1,:) = I2;
                range(end+1,:) = normrnd(1,sigma)*norm(I2-lidar_frame);
        end
        azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
    end
    
    if (check==1)&&(check1==1)&&(check2~=1)
        range_z = norm(I-lidar_frame);
        range_x = norm(I1-lidar_frame);
        if(range_x>range_z)
            intersect_point(end+1,:) = I;
            range(end+1,:) = normrnd(1,sigma)*norm(I-lidar_frame);
            azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
        else
            intersect_point(end+1,:) = I1;
            range(end+1,:) = normrnd(1,sigma)*norm(I1-lidar_frame);
            azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
        end
    end

    if (check==1)&&(check1~=1)&&(check2==1)
        range_z = norm(I-lidar_frame);
        range_y = norm(I2-lidar_frame);
        if(range_y>range_z)
            intersect_point(end+1,:) = I;
            range(end+1,:) = normrnd(1,sigma)*norm(I-lidar_frame);
            azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
        else
            intersect_point(end+1,:) = I2;
            range(end+1,:) = normrnd(1,sigma)*norm(I2-lidar_frame);
            azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
        end
    end

    if (check~=1)&&(check1==1)&&(check2==1)
        range_y = norm(I2-lidar_frame);
        range_x = norm(I1-lidar_frame);
        if(range_y>range_x)
            intersect_point(end+1,:) = I1;
            range(end+1,:) = normrnd(1,sigma)*norm(I1-lidar_frame);
            azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
        else
            intersect_point(end+1,:) = I2;
            range(end+1,:) = normrnd(1,sigma)*norm(I2-lidar_frame);
            azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
        end
    end


    if (check==1)&&(check1~=1)&&(check2~=1)
        intersect_point(end+1,:) = I;
        range(end+1,:) = normrnd(1,sigma)*norm(I-lidar_frame);
        azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
    end
    if (check~=1)&&(check1==1)&&(check2~=1)
        intersect_point(end+1,:) = I1;
        range(end+1,:) = normrnd(1,sigma)*norm(I-lidar_frame);
        azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
    end
    if (check~=1)&&(check1~=1)&&(check2==1)
        intersect_point(end+1,:) = I2;
        range(end+1,:) = normrnd(1,sigma)*norm(I-lidar_frame);
        azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
    end

%     if (check==1)
%         intersect_point(end+1,:) = I;
%         range(end+1,:) = normrnd(1,sigma)*norm(I-lidar_frame);
%         azi_ele_matrix_exclude(end+1,:) = azi_ele_matrix(i,:);
%     end
   
end


% table = [rad2deg(azi_ele_matrix_exclude(:,1)) rad2deg(azi_ele_matrix_exclude(:,2)) range(:,1)]; %deg deg cm
table = [azi_ele_matrix_exclude(:,1) azi_ele_matrix_exclude(:,2) range(:,1)]; %s n m


frame_matrix = zeros(size(intersect_point(:,1),1),3);
for i=1:size(intersect_point(:,1),1)
    frame_matrix(i,:) = lidar_frame;
end

writematrix(table,'simu_testing.csv') 
%%Print surface and frames
plot3(intersect_point(:,1),intersect_point(:,2),intersect_point(:,3),'o'); %plot the point on plane
% hold on 
% for i=1:size(frame_matrix(:,1),1)
%     plot3([lidar_frame(1) point_on_line_2(i,1)] , [lidar_frame(2) point_on_line_2(i,2)], [lidar_frame(3) point_on_line_2(i,3)],'b-');
%     hold on;
% end
% plot3(vector_lidar_frame(:,1),vector_lidar_frame(:,2),vector_lidar_frame(:,3),'o'); % plot laser vector


hold on
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-5 5]);
ylim([-5 5]);
zlim([-3 3]);
