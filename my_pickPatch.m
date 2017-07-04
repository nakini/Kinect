function my_pickPatch
   figure
   x = [0 1 2];
   y = [0 1 0];
   hGroup = hggroup('ButtonDownFcn',@my_groupCB);
   patch(x,y,'b',...
      'Parent',hGroup,...
      'HitTest','off')
   patch(x+2,y,'b',...
      'Parent',hGroup,...
      'HitTest','off')
   patch(x+3,y,'b',...
      'Parent',hGroup,...
      'HitTest','off')
end

function my_groupCB(src,~)
   s = src.Children;
   set(s,'FaceColor',rand(1,3))
end
