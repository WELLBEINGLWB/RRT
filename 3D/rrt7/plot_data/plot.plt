set terminal postscript eps color enhanced 20
set output "out.eps"
set xrange [0:22]
set yrange [0:22]
set zrange [0:22]

set xlabel "x"
set ylabel "y"
set zlabel "z"
set ticslevel 0

splot "testcase1_obstacle.dat" using 1:2:3 with lines lt 1 lc rgb "#ff0033" lw 3 title 'Obstacle',\
'start_goal.dat' using 1:2:3 with points pt 7 ps 2 lt rgb "#ff9900" title 'Start & Goal',\
'data.dat' using 1:2:3 with lines lt 1 lc rgb "#696969" lw 1 title 'node',\
'path_data.dat' using 1:2:3 with lines lt 1 lc rgb "#191970" lw 5 title 'Path'