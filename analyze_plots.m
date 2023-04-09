function [] = analyze_plots(fig_name)
fig = openfig(fig_name, 'reuse');

%% top plot data
top_plot = fig.Children.Children(2).Children.Children(2);
t1 = top_plot.Children(2).XData;
y1 = top_plot.Children(2).YData;

t2 = top_plot.Children(1).XData;
y2 = top_plot.Children(1).YData;

%% bottom plot data
bottom_plot = fig.Children.Children(1).Children.Children(2);
t3 = bottom_plot.Children(2).XData;
y3 = bottom_plot.Children(2).YData;

t4 = bottom_plot.Children(1).XData;
y4 = bottom_plot.Children(1).YData;

%% step info
step_info1 = stepinfo(y1,t1)
step_info2 = stepinfo(y2,t2)
step_info3 = stepinfo(y3,t3)
step_info4 = stepinfo(y4,t4)

%% error info
max_error31 = max(y3-y1)
max_error42 = max(y4-y2)
end