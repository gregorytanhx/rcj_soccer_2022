float rad2deg(float angle){
  return angle * 180/pi;
}

float deg2rad(float angle){
  return angle * pi/180;
}

double norm(float val, int max, int min){
  return (val - min)/(max - min)
}