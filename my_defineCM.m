function cmHandle = my_defineCM
   cmHandle = uicontextmenu;
   uimenu(cmHandle,'Label','Wider','Callback',@increaseLW);
   uimenu(cmHandle,'Label','Inspect','Callback',@inspectLine);
end
function increaseLW(~,~)
% Increase line width
   h = gco;
   orgLW = h.LineWidth;
   h.LineWidth = orgLW+1;
end
function inspectLine(~,~)
% Open the property inspector
   h = gco;
   inspect(h)
end

%% Example
% plot(rand(1,5),'UIContextMenu',my_defineCM)
