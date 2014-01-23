function myPlot(myVar, name, saveAs) 
% name = string, that you want for title
% myVar = variable that you want to plot
% saveAs = name of the pdf to be produced

figure(1); clf;
plot(myVar);
title(name);
set(gcf, 'PaperPosition', [0 0 25 15]);
set(gcf, 'PaperSize', [25 15]); %Set the paper to have width 25 and height 15.
legend(name);
figureHandle = gcf
set(findall(figureHandle,'type','text'),'fontSize',14,'fontWeight','bold')
saveas(gcf, saveAs, 'pdf')

end