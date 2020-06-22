filename = "../../build-quadrupedrobot-Desktop_Qt_5_12_1_GCC_64bit-Debug/gpPlotfile_trot"
set terminal svg background "white"
set output "/home/cx/Desktop/atti_graph_trot.svg"
set xlabel "时间(单位：秒)"
set ylabel "姿态与偏航角(单位：弧度)"
set title "Trot步态(输入指令为1m/s)" font ",20"
set key bottom right
set grid
plot filename skip 200 u ($1/1000):2 title "滚转角" with lines, "" skip 200 u ($1/1000):3 title "俯仰角" with lines, "" skip 200 u ($1/1000):4 title "偏航角" with lines  lc "red"

set output "/home/cx/Desktop/limbpos_graph_trot.svg"
set multiplot layout 2, 2

set xlabel "时间(单位：秒)"
set ylabel "足端距本体高度(单位：米)"
set title "左前腿" font ",12"
set key top right font ",8"
plot filename skip 200 u ($1/1000):10 title "左前腿" with l lc rgb "0xA0FF0000"

set xlabel "时间(单位：秒)"
set ylabel "足端距本体高度(单位：米)"
set title "右前腿" font ",12"
set key top right font ",8"
plot filename skip 200 u ($1/1000):11 title "右前腿" with l lc rgb "0xA000FF00"

set xlabel "时间(单位：秒)"
set ylabel "足端距本体高度(单位：米)"
set title "左后腿" font ",12"
set key top right font ",8"
plot filename skip 200 u ($1/1000):12 title "左后腿" with l lc rgb "0xA00000FF"

set xlabel "时间(单位：秒)"
set ylabel "足端距本体高度(单位：米)"
set title "右后腿" font ",12"
set key top right font ",8"
plot filename skip 200 u ($1/1000):13 title "右后腿" with l lc rgb "0xA0FF00FF"
