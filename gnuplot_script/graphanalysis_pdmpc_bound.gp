filename = "../../build-quadrupedrobot-Desktop_Qt_5_12_1_GCC_64bit-Debug/gpPlotfile_pdmpc_bound"
set terminal postscript eps color #svg background "white"

set output "/home/cx/Desktop/graph_attitude_pdmpc_bound.eps"
set xlabel "Time(second)"
set ylabel "Attitude(rad)"
set title "PDMPC Controller" font ",20"
set key top right
#set grid
set xrange [4:18]
set yrange [-0.4:0.4]
plot filename u ($1/1000):2 title "Roll" with lines lc rgb "0xFF0000", "" u ($1/1000):3 title "Pitch" with lines lc rgb "0x0000FF"


