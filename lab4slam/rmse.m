% Load the .mat file
data_pred = load('collected_data.mat');
data_gt = load('gt.mat');
% Extract all_idx2num
pred_idx = data_pred.all_idx2num;
gt_idx = data_gt.idx;
gt_landmar_pos = {};
N = length(pred_idx);

% Extract x and y positions for each set of landmarks
for i = 1:length(N)
    landmark_id = find(pred_idx(i) == gt_idx(i));
    gt_landmarks = data_gt.all_landmarks(landmark_id);
    gt_landmarks_pos{i} = gt_landmarks;
end

res = compute_rmse(data_pred.all_landmarks, gt_landmarks_pos);
disp(res);


function rmse = compute_rmse(landmarks1, landmarks2)
    diff = landmarks1 - landmarks2;
    square_diff = diff.^2;
    rmse = sum(square_diff, 'all');
end