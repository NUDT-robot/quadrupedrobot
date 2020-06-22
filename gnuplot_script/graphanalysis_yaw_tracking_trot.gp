filename = "../../build-quadrupedrobot-Desktop_Qt_5_12_1_GCC_64bit-Debug/gpPlotfile_yaw_tracking_trot"
set terminal postscript eps color #svg background "white"
set output "/home/cx/Desktop/graph_yaw_tracking_trot.eps"
set xlabel "Time(second)"
set ylabel "Yaw angle(rad)"
set title "Yaw angle tracking in a trotting gait" font ",20"
set key top left
#set grid
set xrange [4:18]
plot filename skip 200 u ($1/1000):14 title "Command angle" with lines lc "red", "" skip 200 u ($1/1000):4 title "Yaw angle" with lines lc "green"


