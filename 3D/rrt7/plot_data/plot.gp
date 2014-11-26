#set terminal gif animate optimize size
#set output 'anime.gif'

set xrange [0:22]
set yrange [0:22]
set zrange [0:22]

set xlabel "x"
set ylabel "y"
set zlabel "z"
set ticslevel 0

splot "testcase1_obstacle.dat" using 1:2:3 with lines lt rgb "#ff0033" title 'Obstacle',\
'data.dat' using 1:2:3 with lines lt rgb "#696969" lw 2 title 'node',\
'start_goal.dat' using 1:2:3 with points pt 7 ps 2 lt rgb "#ff9900" title 'Start & Goal'
pause 1
reread
