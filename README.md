## warmup_project

# Drive in a square

    High Level Description: Have a robot move forward then turn 90 degrees four times to drive in a square
    
    
    Code Explanation: This code is object oriented, is sets up a node and then publishes to it using the \cmd_vel topic
    It loops through 4 times driving forward and then turning. both driving forward and turning take 10 seconds, 9 of doing the action
    and then it sets the new topic and then waits 1 more second before publishing. 
    To turn 90 degrees, the robot sets the velocity to pi/2 radians divided by the time of the movement, which is 10 seconds.
    
    
    GIF: 
  ![Drive Square](https://github.com/snitkin/warmup_project/blob/203a438dd31e93879e8c278ac01ea55e5c094bac/gifs/drive_square.gif)
    
    
    Challenges: As I begin this project, the biggest challenge is developing a workflow for this class. The speed of my VM and various issues trying to connect to the robot greatly slowdown the speed of my work. It was also challenging to understand that there had to be a pause before the topic was published, I only figured this out when the labsolutions were posted. Additionally, there seemed to be error in the robot turning 90 degrees on subsequent turns, and the source of that error is unknown.

# Person Follower 

    High level description:
        robot rotates towards nearest object and moves towards it proportionally

    Code Explanation:
        The code uses object oriented programming to subscribe to laser scanner with a call back function and publish to command velocity. In the call back function, it takes the array of distances and sets 0 to a larger number to ignore the 0s that represent no object. It takes the min distance as the distance to the person and the and the argmin as the angle to the person and moves towards the person with a speed and angular velocity proportional to the angle and the distance- a nearest distance the robot is willing to get
        
    GIF: 
 
 ![Person Follower](https://github.com/snitkin/warmup_project/blob/9e5a34a3c908108b17eb4b8477d41be1bc13bb8e/gifs/person_follower.gif)

    Challenges: Knowing the proportions to set the speed to is difficult and takes some trial and error. However my main conceptual difficulty is thinking about how to aggregate the sensor readings to account for noise

# Wall Follower

    High level description:
        When far from a wall, go to closest object, when close to a wall, put the wall at 90 degrees and drive forward, adjusting the angle proportionately to keep the wall at 90 degrees

    Code Explanation:
        The code uses object oriented programming to subscribe to laser scanner with a call back function and publish to command velocity. In the call back function, it takes the array of distances and sets 0 to a larger number to ignore the 0s that represent no object. When the object is greater than .3 the following distance, it implements the person follower agorithm to go to the nearest object. When it is close, it checks to see if the angle of the closest object is within 5 degrees of 90. If it is, the robot moves forward, if not it stays still linearly. In both cases, turn proportionately to put the closest angle at 90.

    GIF: 
    
  ![Wall Follower](https://github.com/snitkin/warmup_project/blob/9e5a34a3c908108b17eb4b8477d41be1bc13bb8e/gifs/wall_follower.gif)

    Challenges: 
        Outside turns proved tricky for this robot when it got to far from the wall. This is why I implemented person follower, so it would come back to the wall. Thinking about the angles, how to keep the wall at 90, was intially tricky but once I wrote out the math I realized it was quire simple. 
    
Futrue Work:
    If I had more time, I would work on making all my robot's movements smoother. For drive in a square, each angle is more off from 90 degrees than the one before and I don't understand why. I would spend time investigating this. For person follower and wall follower, I would spend more time calibrating the proportional values for movements to make them flow better. Especially for wall follower, my movements are quite janky. Additionally, I would work on aggregating the sensor data so it could be more robust to random noise.

Takeaways: 
    --Robot movement is not always precise and exact. Codes controlling the reading of sensors and movmenets must take into account measurment and motor error in order to adjust and perform the proper behavior. 

    --Conditional programming is very effective for robots. They will encounter different situations, and an if else statement can be easier than an equation that is robust to all condtions. 
