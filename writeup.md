# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
・filter:
	This step implements the EKF functions to predict x and P. The prediction and the measurement value are updated when new measurements are available, where the F and Q matrices are updated assuming a constatn velocity process. Other parameters (eg., gamma, S, H) are calculated using the EKF functions as needed. 
    
・track management
	This step is to update the track's state to "initialized, tentative, or confirmed" based on track score. If score is too low and/or variace is too high, the track gets deleted.
    
・association
	This step associates a measurement to a track based on Mahalanobis distance. A gating function is used to check if a measurement liew inside a track's gate. An associated measurement and track are deleted to update unassigned measurements and tracks lists.
    
・camera fusion
	In this step, camera sensor measurements are initialized. Accordingly, the Sensor class is updated to apply sensor fusion using the nonlinear camera measurement model and a sensor visibility check to see if the input x of an object can be seen by this sensor.
    

The "track management" was the most difficult part for me. I didn't pay very close attention on the code beyond the part marked as "TO DO".
I had an error saying "'Track' object has no attribute 'id'". Figured this out by asking a mentor (Neha).
Similar issue happened in "measurements" part. Didn't define "sensor" for the Measurement class initially. But quickly got it fixed with the experience from "track management".
Another challenge is to delete track. I initially didn't get the condition right and deleted initialized track whocse track score is low. The Hints in project instructions
helped me to correct my code.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
Intuitively, redudency on available sensors and measurements should help on the predict accuracy and reliability. However, based on my project results, the camera-lida (Step 4) RMSEs are the same as the lidar-only tracking (Step 3). This can potentially be improved by finetuning parameters (e.g., delete_threshold). 
Also, I noticed some non-vehicle objects were fasely detected. The camera detection part should probably be better trained or calibrated.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
In this project, I saw non-vehicle objects (e.g., plants on the roadside, pedestrians) were falsely detected as a vehicle and tracked for a period of time. Although eventually they got deleted. This is one of the challenges can be expected in real-life scenarios, if the shape of the object is very similar to a vehicle and never gets deleted.

### 4. Can you think of ways to improve your tracking results in the future?
· A digitalized environment can help improve accuracy. For example, plants on roadside would be already part of the environment digital map and shouldn't be detected as an vehicle. Also, the curbe will be stored as a boundary to detect vehicles so that pedestirans won't be detected and tracked.
· Improve or finetun the prameters such as delete_threshold and confirmed_threshold to improve accuracy.
· Upgrade the process to use a non-linear motion model.
