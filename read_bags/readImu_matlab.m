function imu = readImu_matlab( imu_topic )

imu_msgs = readMessages(imu_topic);

sz = length(imu_msgs);

imu.t = zeros(1, sz);
imu.i = zeros(1, sz);
imu.a = zeros(3, sz);
imu.w = zeros(3, sz);
imu.q = zeros(4, sz);

for i=1:sz
   imu.t(i) = imu_topic.MessageList(i,1).Time;
   imu.i(i) = imu_msgs{i}.Header.Seq;
   imu.a(:,i) = [imu_msgs{i}.LinearAcceleration.X;
       imu_msgs{i}.LinearAcceleration.Y;
       imu_msgs{i}.LinearAcceleration.Z];
   imu.w(:,i) = [imu_msgs{i}.AngularVelocity.X;
       imu_msgs{i}.AngularVelocity.Y;
       imu_msgs{i}.AngularVelocity.Z;];
   imu.q(:,i) = [imu_msgs{i}.Orientation.X;
       imu_msgs{i}.Orientation.Y;
       imu_msgs{i}.Orientation.Z;
       imu_msgs{i}.Orientation.W;];
end

