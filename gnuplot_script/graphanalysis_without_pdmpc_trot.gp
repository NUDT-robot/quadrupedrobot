filename = "../../build-quadrupedrobot-Desktop_Qt_5_12_1_GCC_64bit-Debug/gpPlotfile_without_pdmpc_trot"
set terminal postscript eps color #svg background "white"
set output "/home/cx/Desktop/graph_attitude_without_pdmpc_trot.eps"

set xlabel "Time(second)"
set ylabel "Attitude(rad)"
set title "Convex MPC Controller" font ",20"
set key top right
#set grid
set xrange [4:7]
set yrange [-0.5:1.2]
plot filename u ($1/1000):2 title "Roll" with lines lc rgb "0xFF0000", "" u ($1/1000):3 title "Pitch" with lines lc rgb "0x0000FF"


