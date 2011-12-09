#PRE=/home/khan
PRE=/localhome/khan
POST=/rascratch/user/sickday

for i in  logs/outdoor/20100308/images/grass  logs/outdoor/20100507/1631/grass  logs/outdoor/20100308/images/asphalt  logs/outdoor/20100507/1629/asphalt  logs/outdoor/20100507/1629/gravel  logs/outdoor/20100507/1630/gravel  logs/outdoor/20100701/images/1120/outdoor_bigtiles  logs/outdoor/20100701/images/1129/outdoor_bigtiles  logs/outdoor/20100701/images/1120/outdoor_smalltiles  logs/outdoor/20100701/images/1123/outdoor_smalltiles
do
  for j in LBP LTP LATP gsurf daisy CCH
  do 
    echo
    echo moving descriptors from: $PRE/$i/$j/ to: $POST/$i/$j
    mv -f $PRE/$i/$j/* $POST/$i/$j
  done
done

