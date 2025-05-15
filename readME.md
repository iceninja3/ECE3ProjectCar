# ECE 3 Project Car to follow a path with infrared sensors

## Basic control flow of our program:
```
void loop () {
readSensors();
calculateError();

pid_output = computePID(error);

setMotorSpeeds(base_speed+pid_output, base_speed-pidoutput);
}
```

## Things car needs to do and can't do yet:
2. When a split in the path is encountered, the car needs to take the left path
3. The car reaches the end of the path, it needs to turn around and come back (this time it takes the right paths since the old left is now right)

## Ideas:
2. Need to do something along the lines of: if (the outer sensors both sense black) -> ignore the right side's black? Or maybe just weight hte left more heavily? hard code the car to turn left?
3. After a few consecutive all-white readings (maybe 5? Shouldn't be too many because then the car might have trouble finding the path again), the car switches "modes" to turn around and come back


Idea to go left pseudocode:

if(splitcounter is more than 10){ set inSplit state to true}

If(the line is in middle of car sensors) { set "insplit" state to false and splitcounter to 0} -> to check if line is in middle of car sensors just see if the sum of the middle three senesors is above some vvalue

if (inSplit is true){ turn left } -> turn left by doing a ChangeWheelSpeed but using our own speed (like ChangeWheelSpeed (-10, 10) or something).
