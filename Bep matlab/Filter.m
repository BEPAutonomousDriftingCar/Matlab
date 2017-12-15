%Determines constants of the butterworth filter(Filters Frequency above 5hz)
[b2, a2] = butter(20, 0.5, 'low');

%Filters IMU Data
A.Data(:,1) = filtfilt(b2,a2,A.Data(:,1));
A.Data(:,2) = filtfilt(b2,a2,A.Data(:,2));
A.Data(:,3) = filtfilt(b2,a2,A.Data(:,3));
%D.Data(:,1) = filtfilt(b2,a2,D.Data(:,1));
