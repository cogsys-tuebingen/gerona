#PRE=/home/khan
PRE=/rascratch/user/sickday
POST=/localhome/khan

for i in  logs/outdoor/20100308/images/grass  logs/outdoor/20100507/1631/grass  logs/outdoor/20100308/images/asphalt  logs/outdoor/20100507/1629/asphalt  logs/outdoor/20100507/1629/gravel  logs/outdoor/20100507/1630/gravel  logs/outdoor/20100701/images/1120/outdoor_bigtiles  logs/outdoor/20100701/images/1129/outdoor_bigtiles  logs/outdoor/20100701/images/1120/outdoor_smalltiles  logs/outdoor/20100701/images/1123/outdoor_smalltiles
do
#  echo copying $i ...
  echo
  echo copying $PRE/$i/*.png to $POST/$i/
  cp $PRE/$i/*.png $POST/$i/
done

