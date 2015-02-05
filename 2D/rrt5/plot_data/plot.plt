set terminal postscript eps color enhanced 20
set output "out.eps"
set xrange [0:22]
set yrange [0:22]
set xlabel "x"
set ylabel "y"

set key outside
set key top right

set size square
plot "testcase1_obstacle.dat" using 1:2 with filledcurves lt rgb "#ff0033" fill solid 0.5 notitle,\
'start_goal.dat' using 1:2 with points pt 7 ps 2 lt rgb "#ff9900" title 'Start & Goal',\
'data.dat' using 1:2 with lines lt 1 lc rgb "#696969" lw 1 title 'node',\
'path_data.dat' using 1:2 with lines lt 1 lc rgb "#ff4500" lw 2 title 'Before',\
'path_data_mod.dat' using 1:2 with lines lt 1 lc rgb "#191970" lw 5 title 'After',\
'Bspline.dat' using 1:2 with lines lt 1 lc rgb "#2CC639" lw 2 title 'Bspline'