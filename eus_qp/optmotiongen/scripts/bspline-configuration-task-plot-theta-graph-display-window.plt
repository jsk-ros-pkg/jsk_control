file = "/tmp/bspline-configuration-task-plot-theta-graph.dat"
plot file u 1:2 t "theta" w l lw 4
rep file u 1:5 t "theta-dot" w l lw 4
if (plot_numerical==1) { rep file u 1:6 t "theta-dot-numerical" w l lw 4 }
rep file u 1:9 t "theta-dot2" w l lw 4
if (plot_numerical==1) { rep file u 1:10 t "theta-dot2-numerical" w l lw 4 }
set grid
rep
pause -1 "hit Enter key"
