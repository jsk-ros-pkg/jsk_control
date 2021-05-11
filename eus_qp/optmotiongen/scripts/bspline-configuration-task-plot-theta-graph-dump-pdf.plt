file = "/tmp/bspline-configuration-task-plot-theta-graph.dat"
pdf_file = sprintf("/tmp/bspline-configuration-task-plot-theta-graph-%s-%s.pdf", plot_trg, joint_name)

set grid

set term pdfcairo enhanced size 4.0in, 3.0in
set output pdf_file

if (plot_trg eq 'theta') {
   plot file u 1:2 t "theta" w l lw 6 lc rgb "red", \
        file u 1:3 t "theta max limit" w l lw 4 lc rgb "dark-red", \
        file u 1:4 t "theta min limit" w l lw 4 lc rgb "dark-red"
   set xlabel "time [s]" font ",14"
   set ylabel "joint angle [rad]" font ",14"
   set title sprintf("%s joint angle", joint_name) font ",20" noenhanced
}
if (plot_trg eq 'theta-dot') {
   plot file u 1:5 t "theta-dot" w l lw 6 lc rgb "green", \
        file u 1:7 t "theta-dot max limit" w l lw 4 lc rgb "dark-green", \
        file u 1:8 t "theta-dot min limit" w l lw 4 lc rgb "dark-green"
   set xlabel "time [s]" font ",14"
   set ylabel "joint velocity [rad/s]" font ",14"
   set title sprintf("%s joint velocity", joint_name) font ",20" noenhanced
}
if (plot_trg eq 'theta-dot2') {
   plot file u 1:9 t "theta-dot2" w l lw 6 lc rgb "blue", \
        file u 1:11 t "theta-dot2 max limit" w l lw 4 lc rgb "dark-blue", \
        file u 1:12 t "theta-dot2 min limit" w l lw 4 lc rgb "dark-blue"
   set xlabel "time [s]" font ",14"
   set ylabel "joint acceleration [rad/s^2]" font ",14"
   set title sprintf("%s joint acceleration", joint_name) font ",20" noenhanced
}
if (plot_trg eq 'bspline') {
   plot file u 1:2 notitle w l lw 6 lc rgb "red"
   replot for [i=0:num_control-1] file u 1:13+i notitle w l lw 3
   set xlabel "time [s]" font ",14"
   set ylabel "joint angle [rad]" font ",14"
   set title sprintf("%s joint angle and bspline function", joint_name) font ",20" noenhanced
}

Y_MAX=GPVAL_Y_MAX
Y_MIN=GPVAL_Y_MIN
set yrange [Y_MIN-(Y_MAX-Y_MIN)*0.05:Y_MAX+(Y_MAX-Y_MIN)*0.05]

set output pdf_file
rep
