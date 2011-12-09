BASE_PATH=/rascratch/user/sickday/logs/outdoor
#BASE_PATH=/home/khan/logs/outdoor
gen=1
for i in 5 #80 100 20 90 30 80 40 70 50 60 10 
do
  for j in 20 15 10 2 #11 12 13 14 15 16 17 18 19 9 3 8 4 7 5 6
  do
    if $gen -eq 1
    then
      src/surf 12 $i $i $j $BASE_PATH/20100308/images/grass
      src/surf 12 $i $i $j $BASE_PATH/20100507/1631/grass
      src/surf 12 $i $i $j $BASE_PATH/20100308/images/asphalt
      src/surf 12 $i $i $j $BASE_PATH/20100507/1629/asphalt
      src/surf 12 $i $i $j $BASE_PATH/20100507/1629/gravel
      src/surf 12 $i $i $j $BASE_PATH/20100507/1630/gravel
      src/surf 12 $i $i $j $BASE_PATH/20100701/images/1120/outdoor_bigtiles
      src/surf 12 $i $i $j $BASE_PATH/20100701/images/1129/outdoor_bigtiles
      src/surf 12 $i $i $j $BASE_PATH/20100701/images/1120/outdoor_smalltiles
      src/surf 12 $i $i $j $BASE_PATH/20100701/images/1123/outdoor_smalltiles
    fi
    src/surf 10 gsurf${j}_${i}x${i} 1 0
    src/surf 10 gsurf${j}_${i}x${i} 1 1
  done
done

