#!/bin/bash

clear

for d in minimal-pn.X HB-vector.X FB-vector.X
do

make -C ${d}
if [ -f /tmp/${d}.production.hex ]
then
diff -ub /tmp/${d}.production.hex ${d}/dist/default/production/${d}.production.hex
else
cp ${d}/dist/default/production/${d}.production.hex /tmp
fi

done

