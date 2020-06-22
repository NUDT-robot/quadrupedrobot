filename = "../../build-quadrupedrobot-Desktop_Qt_5_12_1_GCC_64bit-Debug/gpPlotfile_pdmpc_trot"
set terminal postscript eps color #svg background "white"

set output "/home/cx/Desktop/graph_pdmpc_trot_FL.eps"
set xlabel "Time(second)"
set ylabel "Front left foot force in the z axis (N)"
set title "Troting gait with PDMPC controller(FL)" font ",20"
set key bottom right
#set grid
set xrange [6:8]
plot filename u ($1/1000):20 title "UFMPC" with lines lc rgb "0xA0FF0000", "" u ($1/1000):32 title "PD compensator" with lines lc rgb "0xA000FF00", "" u ($1/1000):44 title "PDMPC" with lines lc rgb "0xA00000FF"

set output "/home/cx/Desktop/graph_pdmpc_trot_FR.eps"
set xlabel "Time(second)"
set ylabel "Front right foot force in the z axis (N)"
set title "Troting gait with PDMPC controller(FR)" font ",20"
set key bottom right
#set grid
set xrange [6:8]
plot filename u ($1/1000):23 title "UFMPC" with lines lc rgb "0xA0FF0000", "" u ($1/1000):35 title "PD compensator" with lines lc rgb "0xA000FF00", "" u ($1/1000):47 title "PDMPC" with lines lc rgb "0xA00000FF"

set output "/home/cx/Desktop/graph_pdmpc_trot_BF.eps"
set xlabel "Time(second)"
set ylabel "Back left foot force in the z axis (N)"
set title "Troting gait with PDMPC controller(BL)" font ",20"
set key bottom right
#set grid
set xrange [6:8]
plot filename u ($1/1000):26 title "UFMPC" with lines lc rgb "0xA0FF0000", "" u ($1/1000):38 title "PD compensator" with lines lc rgb "0xA000FF00", "" u ($1/1000):50 title "PDMPC" with lines lc rgb "0xA00000FF"

set output "/home/cx/Desktop/graph_pdmpc_trot_BR.eps"
set xlabel "Time(second)"
set ylabel "Back right foot force in the z axis (N)"
set title "Troting gait with PDMPC controller(BR)" font ",20"
set key bottom right
#set grid
set xrange [6:8]
plot filename u ($1/1000):29 title "UFMPC" with lines lc rgb "0xA0FF0000", "" u ($1/1000):38 title "PD compensator" with lines lc rgb "0xA000FF00", "" u ($1/1000):50 title "PDMPC" with lines lc rgb "0xA00000FF"

set output "/home/cx/Desktop/graph_attitude_pdmpc_trot.eps"
set xlabel "Time(second)"
set ylabel "Attitude(rad)"
set title "PDMPC controller" font ",20"
set key top right
#set grid
set xrange [4:18]
set yrange [-0.3:0.3]
plot filename u ($1/1000):2 title "Roll" with lines lc rgb "0xFF0000", "" u ($1/1000):3 title "Pitch" with lines lc rgb "0x0000FF"


