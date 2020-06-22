filename = "../../build-quadrupedrobot-Desktop_Qt_5_12_1_GCC_64bit-Debug/gpPlotfile_pronk"
set terminal svg background rgb 'white'
set output "/home/cx/Desktop/atti_graph_pronk.svg"
set xlabel "时间(单位：秒)"
set ylabel "姿态与偏航角(单位：弧度)"
set title "Pronk/Jump步态" font ",20"
set key bottom right
set grid
plot filename skip 200 u ($1/1000):2 title "滚转角" with lines, "" skip 200 u ($1/1000):3 title "俯仰角" with lines, "" skip 200 u ($1/1000):4 title "偏航角" with lines  lc "red"

set output "/home/cx/Desktop/limbpos_graph_pronk.svg"
set xlabel "时间(单位：秒)"
set ylabel "足端距本体高度(单位：米)"
set title "Pronk/Jump步态" font ",20"
set key top right font ",8"
set yrange [0.36 : 0.58]
plot filename skip 200 u ($1/1000):10 title "左前腿" with lp pi 100 ps .8 lc rgb "0xA0FF0000",\
"" skip 200 u ($1/1000):11 title "右前腿" with lp ps .8 pi 100 lc rgb "0xA000FF00",\
"" skip 200 u ($1/1000):12 title "左后腿" with lp ps .8 pi 100 lc rgb "0xA00000FF",\
"" skip 200 u ($1/1000):13 title "右后腿" with lp ps .8 pi 100 lc rgb "0xA0FF00FF"
