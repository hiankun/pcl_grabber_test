# pcl_grabber_test

The code was copied and modifed from:
http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_1:_Installing_and_testing#Testing_.28OpenNI_viewer.29

The main modification was adding the inclusion of ``openni2_grabber.h'' and some code for ASUS Xtion.

# usage

1. Make a directory named *build*, go into it, and type ``cmake ..`` to build.
2. Type ``make`` to build the binary **openniViewer**.

# note

1. The code was a workaround for test purpose.
2. The initial viewpoint of the viewer was inverted, and you might have to roll down the middle button of the mouse to see the point cloud.
