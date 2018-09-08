function command = RC2RolllPitchYawRateThrust( rc_topic  )

rc_msgs=readMessages(rc_topic);

len = length(rc_msgs);
arr = zeros(len,1);
rc_B_t = arr;
rc_B_ax1 = arr;
rc_B_ax2 = arr;
rc_B_ax3 = arr;
rc_B_ax4 = arr;
rc_B_ax5 = arr;
rc_B_ax6 = arr;
rc_B_ax7 = arr;
rc_B_ax8 = arr;
for i=1:len
    rc_B_t(i) = rc_topic.MessageList(i,1).Time;
    rc_B_ax1(i) = rc_msgs{i}.Axes(1);
    rc_B_ax2(i) = rc_msgs{i}.Axes(2);
    rc_B_ax3(i) = rc_msgs{i}.Axes(3);
    rc_B_ax4(i) = rc_msgs{i}.Axes(4);
    rc_B_ax5(i) = rc_msgs{i}.Axes(5);
    rc_B_ax6(i) = rc_msgs{i}.Axes(6);
    rc_B_ax7(i) = rc_msgs{i}.Axes(7);
    rc_B_ax8(i) = rc_msgs{i}.Axes(8);
end

command.t = rc_B_t;

%
% rc variables' range: [-1 1]
% command.roll, command.pitch unit: rad
% RC = 1260.5 * Angle + 1024, RC's range: [1024-660, 1024+660]
%

command.roll = -rc_B_ax2' * 660 / 1260.5;
command.pitch = rc_B_ax1' * 660 / 1260.5;
command.yaw_rate = rc_B_ax4' * 660 / 1260.5;
command.thrust = rc_B_ax3'; % not sure how to map

end
