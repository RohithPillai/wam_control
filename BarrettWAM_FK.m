%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implements the motion of the 7-DoF Barrett WAM^TM robotic arm
% Calculate forward kinematics using twists
% of arm with 12cm marker tool
% NOTE: Much of the code structure follows from the example from recitation 1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function eePoses = BarrettWAM_FK(jtData, tws, gst0, viz)

    % calculate the tooltips xyz coords.
    tool_xyz = ones(size(jtData,1),3);

    for t = 1:size(jtData,1)
        g = calcG(tws, jtData(t, :), gst0);
        tool_xyz(t, :) = g(1:3,4);

    end
    
    if viz
        % save xyz coords. to file
        dlmwrite("tooltip_xyz.txt", tool_xyz, ' ');

        % plot the xyz coords.
        plot3(tool_xyz(:,1), tool_xyz(:,2), tool_xyz(:,3), '*k');
    end
%     hold on;
%     % calculate the normal vector of the whiteboard plane snd draw it
%     p1 = tool_xyz((floor(length(tool_xyz)/2)-1000), :);
%     p2 = tool_xyz(floor(length(tool_xyz)/2), :);
%     p3 = tool_xyz((floor(length(tool_xyz)/2)+1000), :);
%     v1 = p1-p2;
%     v2 = p2-p3;
%     n = -cross(v1, v2);
%     nor=[p2; p2+normr(n)];
%     plot3(nor(:,1),nor(:,2),nor(:,3), 'b','LineWidth',5);
%     axis square;
%     fprintf("Normal:");
%     disp(n);
%     fprintf("Point:");
%     disp(p2);
    
    eePoses = tool_xyz;
end




