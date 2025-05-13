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

## Things car needs to do and can't do yet:*
1. Navigate turns (hard part is navigating extremely sharp turns like donuts)
2. When a split in the path is encountered, the car needs to take the left path
3. The car reaches the end of the path, it needs to turn around and come back (this time it takes the right paths since the old left is now right)

## Ideas:
1.
  - Implement Kd (derivative) which may help with slowing down the turn.
  - When all white is detected maybe just keep turning as you were before until you see the track again (would this  mess up when we go off the track at the end?)
  - Limit delta so the total speed can never be greater than some amount? 2x the base speed? Not sure that this would help much since the car can't even get around the turn at the slowest speed regardless
2. Need to do something along the lines of: if (the outer sensors both sense black) -> ignore the right side's black? Or maybe just weight hte left more heavily? hard code the car to turn left?
3. After a few consecutive all-white readings (maybe 5? Shouldn't be too many because then the car might have trouble finding the path again), the car switches "modes" to turn around and come back
