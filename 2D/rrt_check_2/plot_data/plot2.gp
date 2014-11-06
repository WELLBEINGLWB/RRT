set xrange [0:20]
set yrange [0:10]

#set xtics 1
#set ytics 1

set size ratio 0.5
#set size square

set key outside
set key top right

plot "testcase1_obstacle.dat" using 1:2 with filledcurves lt rgb "#ff0033" fill solid 0.5 title "Obstacle",\
'realtimedata.dat' using 1:2 with lines lt rgb "#696969" lw 2 title "Node",\
'start_goal.dat' using 1:2 with points pt 7 ps 2 lt rgb "#ff9900" title "Start & Goal",\
'path_data.dat' using 1:2 with lines lt rgb "#191970" lw 2 title "Path"
pause 1
reread
