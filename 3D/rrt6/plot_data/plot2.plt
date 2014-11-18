set xrange [0:22]
set yrange [0:22]
set zrange [0:22]

set xlabel "x"
set ylabel "y"
set zlabel "z"
set ticslevel 0

splot "testcase1_obstacle.dat" using 1:2:3 with lines lt rgb "#ff0033" title 'Obstacle',\
'data.dat' using 1:2:3 with lines lt rgb "#696969" lw 2 title "Node",\
'start_goal.dat' using 1:2:3 with points pt 7 ps 2 lt rgb "#ff9900" title "Start & Goal",\
'path_data.dat' using 1:2:3 with lines lt rgb "#ff4500" lw 2 title 'Before',\
'path_data_mod.dat' using 1:2:3 with lines lt rgb "#191970" lw 2 title 'After'