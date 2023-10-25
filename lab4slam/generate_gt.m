id = [5, 18, 17, 15, 4, 13, 10, 11];
landmark_pos = [1.38, 0.48; 2.40, 1.61; 3.07, 0.44; 3.5, 2.0; 3.73, 1.13; 2.74, 2.43; 2.40, 1.61; 0.61, 1.65];
all_landmarks = landmark_pos'
save('gt.mat', 'id', 'all_landmarks');