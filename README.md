**Bronze Challenge**

In summary: Traverse the track, don’t crash into any obstacles, report events wirelessly.​
In detail:

   Assemble your buggy
   Starting and stopping the buggy via wireless control, have it traverse the track twice. Pause for any obstacles, reporting these events to the control PC.
   
Details:

   The track will consist of a line forming a loop with a total track length (perimeter) of at least 3m. 
      The track itself is a line on a background of contrasting colour (such that it the line can be detected by the IR sensors). A light line on a dark background will be used (if you are building a section of         track for testing at home then masking tape on dark card as a background would be suitable). In the weeks leading up to the demo a track will be avilable in the labs.
      The track will form a loop (see image below; the track may not look precisely like this but it will approximate the shape, and in particular you will include at least two right-angle turns). The total             length will be at least 3m.

   The PC control program must:
   
      Provide the user with start and stop buttons that can be used to begin and end the buggy's run on the track.
      Provide an output area that displays telemetry received from the buggy during the run. 
      
   The buggy must:
   
      Start the run on receiving a GO command via WiFi  from the controlling PC
      Stop the run on receiving a STOP command via WiFi from the controlling PC
      Traverse the track twice without derailing, using the IR sensors to follow the line of the track
      Pause for obstacles as detected by the US rangefinder. The stopping distance is up to you (but about 10cm is reasonable).
      Report to the controlling PC when obstacles are detected and cleared (a simple "obstacle seen" message is sufficient, but you may choose to do something more details, e.g. "stopping for obstacle at 5cm   distance") and (periodically) an estimate of distance travelled calculated via the wheel encoder. The reporting does not have to display within the Processing graphics window (you can use the console).

**Silver Challenge**

In addition to the Bronze challenge requirements , in the silver challenge:  

The buggy still traverses the line track from the Bronze challenge. However, its speed is to be controlled through a PID controller. Two modes of speed control are to be implemented: (1) via entering the reference speed into the GUI (should be possible to update at any time during the demo) and (2) via reference object placed in sight of the ultrasonic distance sensor. For the second mode, the buggy’s control strategy should be to keep a constant distance from the object (15 cm). The object is moving forward at varying speeds (no backward motion, so the buggy does not need to reverse at any point). The telemetry reported back to the GUI should contain the distance of the object, the current mode of control, the current reference speed, and the current speed of the buggy.

**Gold Challenge**

**Challenge Description **

Your task is to design an autonomous buggy capable of navigating a more complex line-following course. Your buggy will have to navigate junctions and change behaviour based on tags placed on the course. 

**Tags **

Teams must train their autonomous buggies to recognize and interpret four different April tags. These tags should be readable by the HuskyLens vision sensor. The tags will encode instructions to the buggies during the challenge. Each team must bring their chosen tags to the challenge demo. The positioning of the tags on the course will be done by the challenge assessor. The tags should be used to generate the behaviours below as the buggy drives towards and recognises them. 
 
   Turn left at the next junction 
   Turn right at the next junction 
   Observe a Speed Limit (the buggy should adopt a slower speed by the time it reaches this marker) 
   Go as Fast as Possible (the buggy should be moving as fast as it can safely do so once it reaches this sign) 
   
**User Interface** 
The Processing UI must be updated to report on what road signs the buggy has identified, and how they are being interpreted. A portion of the marks will be awarded for creativity in this UI. 
