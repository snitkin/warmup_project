# warmup_project

Drive in a suqare
    High Level Description: Have a robot move forward then turn 90 degrees four times to drive in a square
    
    
    Code Explanation: This code is object oriented, is sets up a node and then publishes to it using the \cmd_vel topic
    It loops through 4 times driving forward and then turning. both driving forward and turning take 10 seconds, 9 of doing the action
    and then it sets the new topic and then waits 1 more second before publishing. 
    To turn 90 degrees, the robot sets the velocity to pi/2 radians divided by the time of the movement, which is 10 seconds.
    
    
    GIF: ![gif of driving in a square](gifs/square.gif)
    
    
    Challenges: As I begin this project, the biggest challenge is developing a workflow for this class. The speed of my VM and various issues trying to connect to the robot greatly slowdown the speed of my work. It was also challenging to understand that there had to be a pause before the topic was published, I only figured this out when the labsolutions were posted. Additionally, there seemed to be error in the robot turning 90 degrees on subsequent turns, and the source of that error is unknown.


Futrue Work:
Takeaways: 