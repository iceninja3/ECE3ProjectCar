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
5/23 Ideas (fix arch, test full runs to make sure they're consistent):
- Test split again. See if we can detect arch or not in the first place.
- Tweak turnSpeed to see if jump, arch, and split can use a common value? If not --> try the weights method

## Current Status:
S-turn: Good
Split: Good on way there and back
Arch: 
- Not sure if we are even detecting the arch split in the first place. I think we are? For a little bit at least
Endpiece/Crosspiece: Stops pretty consistently. On the way back it occasionally doesn't stop? Might not be an issue anymore? Need to test more.

## Ideas:


Idea to go left pseudocode:

if(splitcounter is more than 10){ set inSplit state to true}

If(the line is in middle of car sensors) { set "insplit" state to false and splitcounter to 0} -> to check if line is in middle of car sensors just see if the sum of the middle three senesors is above some vvalue

if (inSplit is true){ turn left } -> turn left by doing a ChangeWheelSpeed but using our own speed (like ChangeWheelSpeed (-10, 10) or something).
