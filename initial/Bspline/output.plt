set xlabel "x"
set ylabel "y"
set zlabel "z"
set ticslevel 0

splot "rawdata.dat" using 1:2:3 with lines lt rgb "#ff0033" lw 3 title 'plot'
replot 'data.dat' using 1:2:3 with lines lt rgb "#696969" lw 1 title 'spline'