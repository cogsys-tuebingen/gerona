BASE_PATH=/localhome/khan/logs/outdoor
#BASE_PATH=/rascratch/user/sickday/logs/outdoor
#BASE_PATH=/home/khan/logs/outdoor
gen=0
for i in 10 #100 80 90 30 80 40 70 50 60 100 20 5 
do
  for j in 19 #5 18 6 11 12 13 14 16 17 18 19 10 2 3 4 8 9 20 15 5 7 
  do
#    if $gen -eq 1
#    then
      src/surf 30 $i $i $j $BASE_PATH/20100308/images/grass
      src/surf 30 $i $i $j $BASE_PATH/20100507/1631/grass
      src/surf 30 $i $i $j $BASE_PATH/20100308/images/asphalt
      src/surf 30 $i $i $j $BASE_PATH/20100507/1629/asphalt
      src/surf 30 $i $i $j $BASE_PATH/20100507/1629/gravel
      src/surf 30 $i $i $j $BASE_PATH/20100507/1630/gravel
      src/surf 30 $i $i $j $BASE_PATH/20100701/images/1120/outdoor_bigtiles
      src/surf 30 $i $i $j $BASE_PATH/20100701/images/1129/outdoor_bigtiles
      src/surf 30 $i $i $j $BASE_PATH/20100701/images/1120/outdoor_smalltiles
      src/surf 30 $i $i $j $BASE_PATH/20100701/images/1123/outdoor_smalltiles
#    fi
    src/surf 32 ltp${j}_${i}x${i} 1 0
    src/surf 32 ltp${j}_${i}x${i} 1 1
    echo "done for this pattern"
  done
  echo "done for this pattern"
done

