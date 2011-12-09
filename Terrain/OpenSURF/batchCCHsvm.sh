BASE_PATH=/rascratch/user/sickday/logs/outdoor
#BASE_PATH=/home/khan/logs/outdoor
gen=0
for i in 10 5 #90 30 80 40 70 50 60 100 20 
do
#if 1
#then
  src/surf 50 $i $i $BASE_PATH/20100308/images/grass
  src/surf 50 $i $i $BASE_PATH/20100507/1631/grass
  src/surf 50 $i $i $BASE_PATH/20100308/images/asphalt
  src/surf 50 $i $i $BASE_PATH/20100507/1629/asphalt
  src/surf 50 $i $i $BASE_PATH/20100507/1629/gravel
  src/surf 50 $i $i $BASE_PATH/20100507/1630/gravel
  src/surf 50 $i $i $BASE_PATH/20100701/images/1120/outdoor_bigtiles
  src/surf 50 $i $i $BASE_PATH/20100701/images/1129/outdoor_bigtiles
  src/surf 50 $i $i $BASE_PATH/20100701/images/1120/outdoor_smalltiles
  src/surf 50 $i $i $BASE_PATH/20100701/images/1123/outdoor_smalltiles
#fi
  src/surf 53 cch${i}x${i} 1 0
  src/surf 53 cch${i}x${i} 1 1
done

