% Load the .mat file
data_pred = load('collected_data.mat');
data_gt = load('gt_lab.mat');

% Extract all_idx2num
pred_idx = data_pred.all_idx2num';
% gt_idx = data_gt.id;
gt_idx = data_gt.numeric_ids;
gt_landmarks_pos = cell(1, length(pred_idx));
N = length(pred_idx);

% Extract x and y positions for each set of landmarks
for i = 1:N
    landmark_idx = find(pred_idx(i) == gt_idx);
    if ~isempty(landmark_idx)
        gt_landmarks = data_gt.all_landmarks(:, landmark_idx);
        gt_landmarks_pos{i} = gt_landmarks;
    else
        % gt_landmarks_pos{i} = NaN(size(data_gt.all_landmarks, 1), 1); % Use NaN for unmatched landmarks
       gt_landmarks_pos{i} = [0; 0];
    end
end

% Convert cell to matrix for RMSE computation
gt_landmarks_matrix = cell2mat(gt_landmarks_pos);

res = compute_rmse(data_pred.all_landmarks, gt_landmarks_matrix);
disp(res);

function rmse = compute_rmse(landmarks1, landmarks2)
    diff = landmarks1 - landmarks2;
    square_diff = diff.^2;
    mse = mean(square_diff, 'all');
    rmse = sqrt(mse);
end
