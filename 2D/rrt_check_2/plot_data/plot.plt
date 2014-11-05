set xrange [0:20]
set yrange [0:20]
set size square
set key outside
set key top right

plot "testcase1_obstacle.dat" using 1:2 with filledcurves lt rgb "#ff0033" fill solid 0.5 title "Obstacle",\
'realtimedata.dat' using 1:2 with lines lt rgb "#696969" lw 2 title "Node",\
'single.dat' using 1:2 with lines lt rgb "#1E90FF" lw 4 title "NEW Created Branch",\
'start_goal.dat' using 1:2 with points pt 7 ps 2 lt rgb "#ff9900" title "Start & Goal",\
'samplenow.dat' using 1:2 with points pt 13 ps 1 lt rgb "#2E8B57" title "NOW SamplePoint",\
'samplenext.dat' using 1:2 with points pt 13 ps 2 lt rgb "#2F4F4F" title "NEXT SamplePoint",\
'path_data.dat' using 1:2 with lines lt rgb "#191970" lw 2 title "Path"