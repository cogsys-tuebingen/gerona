#BASE_PATH=/rascratch/user/sickday/logs/outdoor
BASE_PATH=/localhome/khan/logs/outdoor
gen=0
for i in 10 #80 90 30 80 40 70 50 60 100 5 20 
do
for j in 1.9 #1.0 0.1 0.2 1.8 0.3 1.7 0.4 1.6 0.5 1.5 0.6 1.4 0.7 1.3 0.8 1.2 0.9 1.1 #10 2 9 3 8 4 7 5 6 11 12 13 14 15 16 17 18 19 20 
do
#if $gen -eq 1
#then
  src/surf 40 $i $i $j $BASE_PATH/20100308/images/grass
  src/surf 40 $i $i $j $BASE_PATH/20100507/1631/grass
  src/surf 40 $i $i $j $BASE_PATH/20100308/images/asphalt
  src/surf 40 $i $i $j $BASE_PATH/20100507/1629/asphalt
  src/surf 40 $i $i $j $BASE_PATH/20100507/1629/gravel
  src/surf 40 $i $i $j $BASE_PATH/20100507/1630/gravel
  src/surf 40 $i $i $j $BASE_PATH/20100701/images/1120/outdoor_bigtiles
  src/surf 40 $i $i $j $BASE_PATH/20100701/images/1129/outdoor_bigtiles
  src/surf 40 $i $i $j $BASE_PATH/20100701/images/1120/outdoor_smalltiles
  src/surf 40 $i $i $j $BASE_PATH/20100701/images/1123/outdoor_smalltiles
#fi
  src/surf 42 latp${j}_${i}x${i} 1 0
  src/surf 42 latp${j}_${i}x${i} 1 1
echo "done for this pattern"
echo "done for this pattern"
done
done

