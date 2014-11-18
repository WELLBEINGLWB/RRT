set xrange [0:22]
set yrange [0:22]
set size square
plot "testcase1_obstacle.dat" using 1:2 with filledcurves lt rgb "#ff0033" fill solid 0.5 notitle,\
'path_data.dat' using 1:2 with lines lt rgb "#696969" lw 1 title 'Before',\
'path_data_mod.dat' using 1:2 with lines lt rgb "#191970" lw 2 title 'After'

pause 1
reread