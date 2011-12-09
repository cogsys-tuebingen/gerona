#PRE=/home/khan
PRE=/rascratch/user/sickday

for i in  $PRE/logs/outdoor/20100308/images/grass  $PRE/logs/outdoor/20100507/1631/grass  $PRE/logs/outdoor/20100308/images/asphalt  $PRE/logs/outdoor/20100507/1629/asphalt  $PRE/logs/outdoor/20100507/1629/gravel  $PRE/logs/outdoor/20100507/1630/gravel  $PRE/logs/outdoor/20100701/images/1120/outdoor_bigtiles  $PRE/logs/outdoor/20100701/images/1129/outdoor_bigtiles  $PRE/logs/outdoor/20100701/images/1120/outdoor_smalltiles  $PRE/logs/outdoor/20100701/images/1123/outdoor_smalltiles
do
  cd $i 
  echo $i
  mkdir LBP 
  mkdir LTP 
  mkdir LATP 
  mkdir gsurf
  mkdir gdaisy 
  mkdir CCH
  mv *lbp* LBP/ 
  mv *ltp* LTP/
  mv *latp* LATP/
  mv *gsurf* gsurf/ 
  mv *gdaisy* gdaisy/ 
  mv *cch* CCH/
done

