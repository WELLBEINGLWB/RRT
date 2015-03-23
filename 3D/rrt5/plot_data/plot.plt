set xrange [0:15.6]
set yrange [0:11.6]
set zrange [0:10.6]

set xlabel "x"
set ylabel "y"
set zlabel "z"
set ticslevel 0

splot "testcase1_obstacle.dat" using 1:2:3 with lines lt rgb "#ff0033" lw 3 title 'Obstacle'
replot 'start_goal.dat' using 1:2:3 with points pt 7 ps 2 lt rgb "#ff9900" title 'Start & Goal'
replot 'data.dat' using 1:2:3 with lines lt rgb "#696969" lw 1 title 'node'
replot 'path_data.dat' using 1:2:3 with lines lt rgb "#191970" lw 2 title 'Path'