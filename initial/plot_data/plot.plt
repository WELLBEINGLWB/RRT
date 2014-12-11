set xrange [0:22]
set yrange [0:22]
set size square
plot "testcase1_obstacle.dat" using 1:2 with filledcurves lt rgb "#ff0033" fill solid 0.5 notitle,\
'start_goal.dat' using 1:2 with line lt rgb "#ff9900" title 'Start & Goal'