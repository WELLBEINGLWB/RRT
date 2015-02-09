#set terminal postscript eps color enhanced 20
#set output "out.eps"
set xrange [0:22]
set yrange [0:22]
set xlabel "x"
set ylabel "y"

set key outside
set key top right

set size square
plot 'path_data.dat' using 1:2 with points pt 7 ps 0.78 lt rgb "#ff9900" title 'cut node',\
'path.dat' using 1:2 with lines lt 1 lc rgb "#696969" lw 1 title 'path'