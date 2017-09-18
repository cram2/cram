CRAM Tutorial System
===

This repository holds three ROS packages as examples for fundamental CRAM components:

  * **cram_tutorial_executive**: High-Level executive that holds the top-level plans to be executed by the robot. These plans are the highest authority during plan execution when it comes to decision making. They interface with the high-level plan library.
  * **cram_tutorial_planlib**: Plan library holding high-level robot plans. These plans don't directly interface with the robot, but describe (on a fairly high level) a set of *primitive* atomic robot plans. The plans consist of decision-making components and an interface with the lower-level process modules.
  * **tutorial_process_module**: Process modules define atomic low-level actions that result in interacting with the *outside world*, i.e. sending commands to the robot and reading sensor values. They prepare low-level information so that higher-level plans can operate on that information.

In its basic form, the tutorial components implement a simulated perception-, manipulation-, and knowledge base querying interface. As these are some of the most important concepts in robotics, the use of CRAM is explained on their example.
