COMP 4180, Assignment 1
    Kyle Ahn, 
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

