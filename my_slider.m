function my_slider()
hfig = figure();
slider = uicontrol('Parent', hfig,'Style','slider',...
         'Units','normalized',...
         'Position',[0.3 0.5 0.4 0.1],...
         'Tag','slider1',...
         'UserData',struct('val',0,'diffMax',1),...
         'Callback',@slider_callback);
     
button = uicontrol('Parent', hfig,'Style','pushbutton',...
         'Units','normalized',...
         'Position',[0.4 0.3 0.2 0.1],...
         'String','Display Difference',...
         'Callback',@button_callback);
end

function slider_callback(hObject,eventdata)
	sval = hObject.Value;
	diffMax = hObject.Max - sval;
	data = struct('val',sval,'diffMax',diffMax);
	hObject.UserData = data;
	% For R2014a and earlier: 
	% sval = get(hObject,'Value');  
	% maxval = get(hObject,'Max');  
	% diffMax = maxval - sval;      
	% data = struct('val',sval,'diffMax',diffMax);   
	% set(hObject,'UserData',data);   

end

function button_callback(hObject,eventdata)
	h = findobj('Tag','slider1');
	data = h.UserData;
	% For R2014a and earlier: 
	% data = get(h,'UserData'); 
	display([data.val data.diffMax]);
end
