COMP 4180, Assignment 1
    Kyle Ahn, 7794966
    Goutham Dhannapuneni, 7814509


To Compile:

    In terminal:
    type <cmake .> to create Makefile
    type <make clean> to remove the compiled files and <make> for new compilation

To Run:

    type <./vision> to run the file


NOTES:

    Windows:

        Ball Finder window
            - This window is for selecting the hsv value of the ball(orange)
            - The video shown in this window is the hsv image

        Ball Control window
            - This window is for adjusting the hsv value of the ball if the results are not satisfactory with the click
            - can see the video of the extracted ball.
            - as we change the values of the hsv the video will also update
     
        Field Finder window
            - This window is for selecting the hsv value of the field(green)

        Field Control window
            - This window is for adjusting the hsv value of the field if the results are not satisfactory with the click.
            - can see the video of the extracted field.
            - as we change the values of the hsv the video will also update
     
        Line Control window
            - This window is for adjusting the hsv value of the lines in the field.
            - can see the video of the extracted lines.
            - as we change the values of the hsv the video will also update

        Output window
            - This window will show the final result with the ball and the lines drawn.


--------------------------------------------
Possible bugs on LSD
--------------------------------------------
- Although LSD works quite well, there seems to be a few bugs on merging colinear lines depending on the orientation of the camera. If the position of the camera is too low, then it sometimes merges wrong lines. However, this problem will be fixed by assignment 2 taking robot's position into consideration.

- Unfortunately, centre circle detection was not able to be completed.

------------------------------------------
Contribution
------------------------------------------
Kyle Ahn

 I have mainly worked on LSD part of the assignment. I have used LineSegmentDetector class that is provided by opencv library to extract the lines into vector<Vec4f>. Later, I have converted these Vec4f into KeyLine just to make it clear that vec4f represents line segment. The detailed description of each function is included in the source code. All the codes in vision.cpp is written by Kyle and Goutham.

