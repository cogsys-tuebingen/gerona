#PRE=/home/khan
PRE=/rascratch/user/sickday
for i in 100 90 80 70 60 50 #40 30 30 20
do
#for j in 10 2 9 3 8 4 7 5 6
#do
if 0
then
  ./daisy -g $i -b $PRE/logs/outdoor/20100308/images/grass
  ./daisy -g $i -b $PRE/logs/outdoor/20100507/1631/grass
  ./daisy -g $i -b $PRE/logs/outdoor/20100308/images/asphalt
  ./daisy -g $i -b $PRE/logs/outdoor/20100507/1629/asphalt
  ./daisy -g $i -b $PRE/logs/outdoor/20100507/1629/gravel
  ./daisy -g $i -b $PRE/logs/outdoor/20100507/1630/gravel
  ./daisy -g $i -b $PRE/logs/outdoor/20100701/images/1120/outdoor_bigtiles
  ./daisy -g $i -b $PRE/logs/outdoor/20100701/images/1129/outdoor_bigtiles
  ./daisy -g $i -b $PRE/logs/outdoor/20100701/images/1120/outdoor_smalltiles
  ./daisy -g $i -b $PRE/logs/outdoor/20100701/images/1123/outdoor_smalltiles
fi
#  for k in 10 20 30
#  do
    ./daisy -g $i -w
#  done
#done
done

