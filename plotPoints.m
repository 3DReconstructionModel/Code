p0_sum = 0.0;
for i=1:1:size(p0,2)
    p0_sum = p0_sum + p0(4,i);
end
p0_med = p0_sum / size(p0,2);
p0b = p0 ./ p0(4,:);% * abs(p0_med);

p1_sum = 0.0;
for i=1:1:size(p1,2)
    p1_sum = p1_sum + p1(4,i);
end
p1_med = p1_sum / size(p1,2)
p1b = p1 ./ p1(4,:);% * abs(p1_med);

p2_sum = 0.0;
for i=1:1:size(p2,2)
    p2_sum = p2_sum + p2(4,i);
end
p2_med = p2_sum / size(p2,2)
p2b = p2 ./ p2(4,:);% * abs(p2_med);

p3_sum = 0.0;
for i=1:1:size(p3,2)
    p3_sum = p3_sum + p3(4,i);
end
p3_med = p3_sum / size(p3,2)
p3b = p3 ./ p3(4,:);% * abs(p3_med);

p4_sum = 0.0;
for i=1:1:size(p4,2)
    p4_sum = p4_sum + p4(4,i);
end
p4_med = p4_sum / size(p4,2)
p4b = p4 ./ p4(4,:);% * abs(p4_med);

p5_sum = 0.0;
for i=1:1:size(p5,2)
    p5_sum = p5_sum + p5(4,i);
end
p5_med = p5_sum / size(p5,2)
p5b = p5 ./ p5(4,:);% * abs(p5_med);

p6_sum = 0.0;
for i=1:1:size(p6,2)
    p6_sum = p6_sum + p6(4,i);
end
p6_med = p6_sum / size(p6,2)
p6b = p6 ./ p6(4,:);% * abs(p6_med);

p7_sum = 0.0;
for i=1:1:size(p7,2)
    p7_sum = p7_sum + p7(4,i);
end
p7_med = p7_sum / size(p7,2)
p7b = p7 ./ p7(4,:);% * abs(p7_med);

p8_sum = 0.0;
for i=1:1:size(p8,2)
    p8_sum = p8_sum + p8(4,i);
end
p8_med = p8_sum / size(p8,2)
p8b = p8 ./ p8(4,:);% * abs(p8_med);

p9_sum = 0.0;
for i=1:1:size(p9,2)
    p9_sum = p9_sum + p9(4,i); 
end
p9_med = p9_sum / size(p9,2)
p9b = p9 ./ p9(4,:);% * abs(p9_med);


plotCameras
%plot3(p0b(1,:), p0b(2,:), p0b(3,:), '.b')
%plot3(p2(1,:), p2(2,:), p2(3,:), '.r')
plot3(p1b(1,:), p1b(2,:), p1b(3,:), '.m')
%plot3(p2b(1,:), p2b(2,:), p2b(3,:), '.r')
plot3(p3b(1,:), p3b(2,:), p3b(3,:), '.g')
%plot3(p4b(1,:), p4b(2,:), p4b(3,:), '.k')
plot3(p5b(1,:), p5b(2,:), p5b(3,:), '.c')
plot3(p6b(1,:), p6b(2,:), p6b(3,:), '.k')
%plot3(p7b(1,:), p7b(2,:), p7b(3,:), '.r')
plot3(p8b(1,:), p8b(2,:), p8b(3,:), '.c')
plot3(p9b(1,:), p9b(2,:), p9b(3,:), '.c')

%plot3(p7(1,:), p7(2,:), p7(3,:), '.c')
axis equal
xlim([-1, 1])
ylim([-1, 1])
zlim([0, 5])
% figure;
% p0 = p0';
% p1 = p1';
% p2 = p2';
% p3 = p3';
% p4 = p4';
% p5 = p5';
% p6 = p6';
% p7 = p7';
% p8 = p8';
% p9 = p9';
% points = [p0; p1; p2; p3; p4; p5; p6; p7; p8; p9];
% pcshow(points)
% axis equal
% xlim([0, 100])
% ylim([0, 200])
% zlim([-400, 0])
