# CSC 412 Assignment 03
Human Computer Interaction - Jeffery White

- The only changes made to the code are in the face_tracker.cc file, which is in this repository.

# Methodology

I used a basic 'delta' calculation to get at detecting the head movements, smile, and surprise. So each time through the main loop we can choose to update 'previous' values and then during the main loop compare these values. Threshold values are set in the globals (which can be tweaked as necessary).

For the head movements I assumed a two step for the movement, so no is 1 left 1 right, yes is 1 up 1 down, indian yes is 1 left roll 1 right roll, and the reverse since the order of these events can be either or.

For the Head movements we update the previous values every 5 passes through the loop, giving us a 'window' for detecing a longer movement, when we hit the threshold (for left, up, etc.) triggers are set. These triggers are compared each time through the loop and if successful the value is output.

For smile and surprise I took 4 points and compueted a distance measurement between them. For this routine the code updates previous values every other time through the loop, since these actions are faster than head movements. Each time we compare previous to current and if the threshold is hit, the program outputs the action taken.

# Results

I found that this worked out pretty well, I'm able to on my machine get the output for each head movement, smile, and surprise and it does hold up pretty well to 'jittery' movements, so was generally resistant to odd head motions.

Due to the loop based method I'm employing there can be some odd issues, for example it is possible to get a 'up motion' at the tail end of the 5 loop motion and then have that action basically 'forgotten' before you complete the down motion after the system triggers a reset of the previous values.

One other thing I did note is that my approach is pretty dependent upon 'how close' you are to the screen, at the standard distance it worked out really well, but if I backed off from the camera the delta thresholds don't work quite as well. I haven't looked too much at a tweak for this based on the length of time we had for this, but I think working out the thresholds to be 'percentage' based might help with that condition.

So it short, it works but it's not by any means flawless.
