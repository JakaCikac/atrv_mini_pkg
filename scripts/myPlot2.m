function myPlot2(myVar1,myVar2, name, legend1, legend2, saveAs) 
% name = string, that you want for title
% myVar1 = variable that you want to plot
% myVar2 = second variable to plot
% saveAs = name of the pdf to be produced

figure(1); clf;
plot(myVar1);
hold on;
plot(myVar2, 'r');
title(name);
set(gcf, 'PaperPosition', [0 0 25 15]);
set(gcf, 'PaperSize', [25 15]); %Set the paper to have width 25 and height 15.
legend(legend1, legend2);
figureHandle = gcf
set(findall(figureHandle,'type','text'),'fontSize',14,'fontWeight','bold')
saveas(gcf, saveAs, 'pdf')

end