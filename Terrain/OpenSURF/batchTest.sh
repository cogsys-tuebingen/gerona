for i in 80 70 60 50 40 30 30 20 
do
  src/surf 14 $i $i /home/khan/logs/outdoor/20100308/images/grass
  src/surf 14 $i $i /home/khan/logs/outdoor/20100507/1631/grass
  src/surf 14 $i $i /home/khan/logs/outdoor/20100308/images/asphalt
  src/surf 14 $i $i /home/khan/logs/outdoor/20100507/1629/asphalt
  src/surf 14 $i $i /home/khan/logs/outdoor/20100507/1630/gravel
  for k in 10 15 20 25 30
  do
    src/surf 15 $k $i $i 
  done
done
