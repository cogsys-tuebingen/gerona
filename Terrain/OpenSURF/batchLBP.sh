for i in 80 70 60 50 40 30 30 20 
do
  src/surf 20 $i $i /home/khan/logs/outdoor/20100308/images/grass
  src/surf 20 $i $i /home/khan/logs/outdoor/20100507/1631/grass
  src/surf 20 $i $i /home/khan/logs/outdoor/20100308/images/asphalt
  src/surf 20 $i $i /home/khan/logs/outdoor/20100507/1629/asphalt
  src/surf 20 $i $i /home/khan/logs/outdoor/20100507/1629/gravel
  src/surf 20 $i $i /home/khan/logs/outdoor/20100507/1630/gravel
  src/surf 20 $i $i /home/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles
  src/surf 20 $i $i /home/khan/logs/outdoor/20100701/images/1126/outdoor_bigtiles
  src/surf 20 $i $i /home/khan/logs/outdoor/20100701/images/1120/outdoor_smalltiles
  src/surf 20 $i $i /home/khan/logs/outdoor/20100701/images/1123/outdoor_smalltiles

  for k in 10 15 20 25 30
  do
    src/surf 21 $k $i $i 
  done
done
