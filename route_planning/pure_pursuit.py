//This is a sudo code for pure pursuit 
//input: array of turning points
//output: double linear_velocity, double angualr_velocity

//helper functions
  
//a function that generate an array of vectors from the array of turning points
genVectors(){
 
}

//a function that finds the point on the path with shortest distance to current location
//p1,p2 are the endpoints of the current vector segment we are trying to identify a point on
//cur is our current location
genBase(p1,p2){
  if(dotProduct(p1->p2,p1->cur)<0){
    return p1
  }
  else if(dotProduct(p2->p1,p2->cur)<0){
    //go to examine vector(p2,p3)
    genbase(p2,p3)
  }
  else{
  base.x=(cur.x-p1.x)*(p2.x-p1.x)/dis(p1,p2)
  }
}

//a function that dynamically updates the look_ahead_distance strlen
strlen_update(){
  strlen=20
  return strlen
}


//find the point that we are looking up to, look ahead point
genTarget(strlen, curVector){
  disRemain=dis(base,next closest turning point)
  if(disRemain>=strlen){
    return target_loc=base+curVector/curVectorLength*strlen
  }
  else {
    genTarget(strlen-disRemain,curVector->next)
  }
}

//direct distance between two given points
dis(p1,p2){
  return((p1.x-p2.x)^2+(p1.y-p2.y)^2)
}

//main
main{
  
}
