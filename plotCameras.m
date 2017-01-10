T21 = T0; T31 = T1; T41 = T2; T51 = T3;
T32 = T4; T42 = T5; T52 = T6;
T43 = T7; T53 = T8;
T54 = T9;

figure
hold on
axis equal
plotCamera('Location', [0,0,0],     'Orientation', eye(3),        'Size', 0.2, 'Label', 'T11',  'Color', [0,0,1])
% plotCamera('Location', T21(1:3, 4), 'Orientation', T21(1:3, 1:3), 'Size', 0.2, 'Label', 'T21b', 'Color', [0,1,0])
% plotCamera('Location', T31(1:3, 4), 'Orientation', T31(1:3, 1:3), 'Size', 0.2, 'Label', 'T31b', 'Color', [0,1,1])
% plotCamera('Location', T41(1:3, 4), 'Orientation', T41(1:3, 1:3), 'Size', 0.2, 'Label', 'T41b', 'Color', [1,0,0])
% plotCamera('Location', T51(1:3, 4), 'Orientation', T51(1:3, 1:3), 'Size', 0.2, 'Label', 'T51b', 'Color', [1,0,1])

T52q = [T52; 0 0 0 1];
T21q = [T21; 0 0 0 1];
T51q = [T51; 0 0 0 1];
T521q = T52q * T21q;

% figure
% hold on
% axis equal
% plotCamera('Location', [0,0,0],       'Orientation', eye(3),          'Size', 0.2, 'Label', 'T11',  'Color', [1,0,0])
% plotCamera('Location', T521q(1:3, 4), 'Orientation', T521q(1:3, 1:3), 'Size', 0.2, 'Label', 'T521', 'Color', [0,0,1])
% plotCamera('Location', T51q(1:3, 4),  'Orientation', T51q(1:3, 1:3),  'Size', 0.2, 'Label', 'T51',  'Color', [0,1,0])


% T21b = T21;
% plotCamera('Location', T21b(1:3, 4), 'Orientation', T21b(1:3, 1:3), 'Size', 0.2, 'Label', 'T21b')
% plotCamera('Location', T31(1:3, 4), 'Orientation', T31(1:3, 1:3), 'Size', 0.2, 'Label', 'T31b', 'Color', [0,0,1])
% plotCamera('Location', T32(1:3, 4), 'Orientation', T32(1:3, 1:3), 'Size', 0.2, 'Label', 'T32')
% T32b = [T32(1:3, 1:3) T32(1:3,4)*-1; T32(4, 1:4)*-1];
% T321 = T32b*T21b;
% plotCamera('Location', T321(1:3, 4), 'Orientation', T321(1:3, 1:3), 'Size', 0.2, 'Label', 'T321', 'Color', [0,1,0])


