id = [12, 18, 2, 17, 15, 1, 3, 10, 14];
landmark_pos = [1.38, 0.48; 3.50, 0.0; 3.07, 0.44; 3.73, 1.13; 3.5, 2.0; 2.74, 2.33; 2.40, 1.61; 0.61, 1.65; 0, 2.0];
all_landmarks = landmark_pos'
save('gt.mat', 'id', 'all_landmarks');