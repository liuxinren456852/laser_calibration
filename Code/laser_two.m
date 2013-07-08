function laser_two(num_scans, test_num, pose_num)

% Generate n laser scans of target
[scans_x, scans_y] = generate_scan('/dev/ttyUSB0',38400, num_scans, test_num, 'l2', pose_num);

% Average laserscans, returning data that falls within the (mean-std) and
% (mean+std) ranges
avg_data = average_scans(scans_x, scans_y, num_scans, test_num, 'l2', pose_num)

% Plot averaged data
plot(avg_data(1,:), avg_data(2,:),'r*')