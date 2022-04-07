# warmup_project

Drive in a suqare
    High Level Description: Have a robot move forward then turn 90 degrees four times to drive in a square
    
    
    Code Explanation: This code is object oriented, is sets up a node and then publishes to it using the \cmd_vel topic
    It loops through 4 times driving forward and then turning. both driving forward and turning take 10 seconds, 9 of doing the action
    and then it sets the new topic and then waits 1 more second before publishing. 
    To turn 90 degrees, the robot sets the velocity to pi/2 radians divided by the time of the movement, which is 10 seconds.
    
    
    GIF: ![gif of driving in a square](gifs/IMG_6325_MOV_AdobeCreativeCloudExpress.gif)
    
    
    Challenges: As I begin this project, the biggest challenge is developing a workflow for this class. The speed of my VM and various issues trying to connect to the robot greatly slowdown the speed of my work. It was also challenging to understand that there had to be a pause before the topic was published, I only figured this out when the labsolutions were posted. Additionally, there seemed to be error in the robot turning 90 degrees on subsequent turns, and the source of that error is unknown.

Person Follower 
    High level description:
        robot rotates towards nearest object and moves towards it proportionally

    Code Explanation:
        The code uses object oriented programming to subscribe to laser scanner with a call back function and publish to command velocity. In the call back function, it takes the array of distances and sets 0 to a larger number to ignore the 0s that represent no object. It takes the min distance as the distance to the person and the and the argmin as the angle to the person and moves towards the person with a speed and angular velocity proportional to the angle and the distance- a nearest distance the robot is willing to get
        
    GIF: ![gif of person follower](gifs/person_follower.gif)

    Challenges: Knowing the proportions to set the speed to is difficult and takes some trial and error. However my main conceptual difficulty is thinking about how to aggregate the sensor readings to account for noise

Wall Follower
    High level description:

    Code Explanation:

    GIF:

    Challenges:
    
Futrue Work:
Takeaways: 