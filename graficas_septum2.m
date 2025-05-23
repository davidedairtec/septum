close all

cell_2 = {0,0.5,12,"Times New Roman",...
            Vdc,"DC Load Voltage ","Voltage [V]",...
            Idc,"DC Load Current","Current [A]",...
            Pdc,"Load Power Consumption ","Power [KW]"}; 
        

cell_ref_comp = {0,0.5,12,"Times New Roman",...
            Vabc,"Grid Voltage","Voltage [V]",...
            Iabc,"Grid Current","Current [A]",... 
            Pabc,"Grid Power","Power [KW]"}
       
Represent_var_Elec_2(cell_2);
print('DC LOAD1.png', '-dpng', '-r300');


Represent_var_Elec_4(cell_ref_comp);
print('GRID1', '-dpng', '-r300');


function Represent_var_Elec_2 (cell)
    
init = cell{1};
endt = cell{2};
tamano = cell{3};
type_text = cell{4};

V = cell{5};
V_Title = cell{6};
V_string = cell{7};

I = cell{8};
I_Title = cell{9};
I_string = cell{10};

Pload = cell{11};
Pload_Title = cell{12};
Pload_string = cell{13};


f0 = figure;
f0.Position = [0 0 900 800];

s = subplot(3,1,1);
p = plot(V);
p.LineWidth = 2;
grid minor;
s.Title.String = V_Title;
s.XLabel.String =" ";
s.YLabel.String = V_string;
s.FontName = type_text;
s.FontSize = tamano;
s.XLim = [init, endt];
%s.YLim = [min(V.Data)*0.90, max(V.Data)*1.1];
s.YLim = [-10, 60];

s = subplot(3,1,2);
p = plot(I);
p.LineWidth = 2;
grid minor;
s.Title.String = I_Title;
s.XLabel.String =" ";
s.YLabel.String = I_string;
s.FontName = type_text;
s.FontSize = tamano;
s.XLim = [init, endt];
s.YLim = [-50, 10000];

s = subplot(3,1,3);
p = plot(Pload);
p.YData = p.YData/1000;
p.LineWidth = 2;
grid minor;
s.Title.String = Pload_Title;
s.XLabel.String =" ";
s.YLabel.String = Pload_string;
s.FontName = type_text;
s.FontSize = tamano;
s.XLim = [init, endt];
s.YLim = [-50, 390000/1000];

end


function Represent_var_Elec_4 (cell)
    
init = cell{1};
endt = cell{2};
tamano = cell{3};
type_text = cell{4};

Vabc = cell{5};
Vabc_Title = cell{6};
Vabc_string = cell{7};

Iabc = cell{8};
Iabc_Title = cell{9};
Iabc_string = cell{10};

Pabc = cell{11};
Pabc_Title = cell{12};
Pabc_string = cell{13};


f0 = figure;
f0.Position = [0 0 900 800];


s = subplot(3,1,1);
p = plot(Vabc);
grid minor;
s.Title.String = Vabc_Title;
s.XLabel.String =" ";
s.YLabel.String = Vabc_string;
s.XLabel.String = " ";
s.FontName = type_text;
s.FontSize = tamano;
s.XLim = [init, endt];
s.YLim = [-400, 340];



s = subplot(3,1,2);
p = plot(Iabc);
grid minor;
s.Title.String = Iabc_Title;
s.XLabel.String =" ";
s.YLabel.String = Iabc_string;
s.FontName = type_text;
s.FontSize = tamano;
s.XLim = [init, endt];
%s.YLim = [s.YLim(1)*1.1, max((Pfc.Data(:,1))/1000)*1.2];
s.YLim = [-9000, 9000];


s = subplot(3,1,3);
p = plot(Pabc);
p.LineWidth = 2;
p.YData = p.YData/1000;
grid minor;
s.Title.String = Pabc_Title;
s.XLabel.String =" ";
s.YLabel.String = Pabc_string;
s.FontName = type_text;
s.FontSize = tamano;
s.XLim = [init, endt];
%s.YLim = [min(Psmes.Data(:,1)/1000), max(Psmes.Data(:,1)/1000)*1.1];
s.YLim = [-5, 1600000/1000];

end

