#!/bin/bash

REPERTORY='tube'

date

if [ -d $REPERTORY ]; then
    cd $REPERTORY
else
    mkdir $REPERTORY && cd $REPERTORY
fi

for i in $(seq 0 4); do
    if [ $i -lt 10 ]; then
        nb="00$i"
    elif [ $i -gt 9 ] && [ $i -lt 100 ]; then
        nb="0$i"
    else
        nb=$i
    fi

    nom="$REPERTORY$nb.png"
    raspistill -o $nom -q 100 -rot 180
done

date
