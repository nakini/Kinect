function my_figDelete(~,~)
yn = questdlg('Delete all figures?',...
    'Figure Menu',...
    'Yes','No','No');
switch yn
    case 'Yes'    
        allfigs = findobj(get(groot,'Children'),'Type','figure' );      
        set(allfigs,'DeleteFcn',[]);
        delete(allfigs)
    case 'No'
        return
end
end
