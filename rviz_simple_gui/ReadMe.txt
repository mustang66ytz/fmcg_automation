This Package provides the shell structure for a simple RVIZ Panel.
Three buttons are provided which print out a simple message onto the
terminal window.

This package is to be used as a shell each time you wish to create a new/blank GUI Panel
in your RVIZ application.

Install QT5 Creator to edit the simple_widget.ui so you can add new buttons/sliders etc.

	sudo apt-get install qtcreator
	sudo apt-get install qt5-default 

To run the panel, build the catkin workspace and source it.
Launch RVIZ and select the panels->add_new_panel menu
from the title bar.
You can then select the SimplePanel object,
re-postition it the RVIZ side-bar and begin using it from there.
	
