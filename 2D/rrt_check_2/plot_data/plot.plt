set xrange [0:20]
set yrange [0:20]

set key outside
set key top right
set xrange [0:20]
set yrange [0:10]

set size ratio 0.5

plot "testcase1_obstacle.dat" using 1:2 with filledcurves lt rgb "#ff0033" fill solid 0.5 title "Obstacle",\
'start_goal.dat' using 1:2 with points pt 7 ps 2 lt rgb "#ff9900" title "Start & Goal"
