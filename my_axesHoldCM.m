function cmHandle = my_axesHoldCM
   cmHandle = uicontextmenu;
   uimenu(cmHandle,'Label','hold on','Callback',@holdOn);
   uimenu(cmHandle,'Label','hold off','Callback',@holdOff);
end
function holdOn(~,~)
   fig = gcbf;
   ax = fig.CurrentAxes;
   hold(ax,'on')
end
function holdOff(~,~)
   fig = gcbf;
   ax = fig.CurrentAxes;
   hold(ax,'off')
end

% % Example:
%Create a bar graph and set the PickableParts property of the Bar objects:
% bar(1:20,'PickableParts','none');
% 
% % Create the context menu for the current axes:
% 
% ax = gca;
% ax.UIContextMenu = my_axesHoldCM
