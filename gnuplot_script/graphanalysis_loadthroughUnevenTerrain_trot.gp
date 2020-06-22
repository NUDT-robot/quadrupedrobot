filename = "../../build-quadrupedrobot-Desktop_Qt_5_12_1_GCC_64bit-Debug/gpPlotfile_load40kg_unevenTerrain_trot"
set terminal postscript eps color#svg background "white"

set output "/home/cx/Desktop/graph_attitude_loadthrough_unevenTerrain_trot.eps"
set xlabel "Time(second)"
set ylabel "rad"
set title "Robot attitude and direction with a 40kg floating load" font ",20"
set key top right
#set grid
set xrange [4:50]
set yrange [-0.3:0.3]
plot filename u ($1/1000):2 title "Roll" with lines lc rgb "0xA0FF0000", "" u ($1/1000):3 title "Pitch" with lines lc rgb "0xA00000FF", "" u ($1/1000):4 title "Yaw" with lines lc rgb "0xA000FF00"


