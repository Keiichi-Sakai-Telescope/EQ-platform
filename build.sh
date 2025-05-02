#!/bin/bash

clear

for d in FB-UB-16F18345.X FB-UB-18F25Q10.X HB-B-16F1825.X HB-B-16LF1825.X HB-UB-16F1825.X HB-UB-16LF1825.X
do

make -C ${d}
grep -B10 ^F000 ${d}/build/default/production/PIC16F*.lst

done

