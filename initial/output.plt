set xlabel "x"
set ylabel "y"
set zlabel "z"
set ticslevel 0
splot "cube.dat" using 1:2:3 with lines lw 5 lt rgb "#696969" title "Obstacle"
