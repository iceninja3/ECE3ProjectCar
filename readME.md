ECE 3 Project Car to follow a path with infrared sensors

Basic control flow of our program:
```
void loop () {
readSensors();
calculateError();

pid_output = computePID(error);

setMotorSpeeds(base_speed+pid_output, base_speed-pidoutput);
}
```

Things car needs to do and can't do yet:
- navigate turns (hard part is navigating extremely sharp turns like donuts)
- when a split in the path is encountered, the car needs to take the left path
- the car reaches the end of the path, it needs to turn around and come back (this time it takes the right paths since the old left is now right)
