Bullet Dice
===========

This Qt example application demonstrates the deployment of the Bullet external
physics library with a QtOpenGL application. 

The application creates several virtual dice and places them on a virtual 
table where the user can manipulate them with touch and accelerometer events. 
Simple cube/plane-based physics are applied to the dice with Bullet. The world 
is rendered with QtOpenGL (QGLWidget) using OpenGL ES 2.

This example application is hosted in Nokia Developer Projects:
- http://projects.developer.nokia.com/gles2phys
- http://projects.developer.nokia.com/gles2phys/wiki

This application has been tested on Harmattan, Symbian Anna, and Windows 7
platforms.

Have fun! 


PREREQUISITES 
-------------------------------------------------------------------------------

- Qt basics
- OpenGL basics


ABOUT BULLET
-------------------------------------------------------------------------------
  
Bullet is an open source physics engine featuring 3D collision detection, soft
body dynamics, and rigid body dynamics.

For more information, see:
- http://bulletphysics.org/

In this project the code files and the license of Bullet are located in
src/bullet/.


IMPORTANT FILES/CLASSES 
-------------------------------------------------------------------------------
 
- Mesh.h/cpp: Very simple 3D mesh loader/container class. The definition of 
  the supported file is described later. 
- PhysicsWidget.h/cpp: Represents the physics world and the view of the
  dice table. Holds most of the code of the application.


RUNNING THE EXAMPLE 
-------------------------------------------------------------------------------
 
Touch the screen to pull the dice towards your finger, and rotate the device 
to let gravity pull the dice.


BUILD & INSTALLATION INSTRUCTIONS 
-------------------------------------------------------------------------------
 
Preparations
~~~~~~~~~~~~ 

Check that you have the latest Qt/Qt SDK installed in the development 
environment and on the device.  


Build & installation instructions using Qt SDK
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open bulletdice.pro: 
   File > Open File or Project, select Pro-file. 

2. Select target(s), for example 'Qt for Symbian Anna (Qt SDK)', and press the
   Finish button. 

3. Press the Run button to build the project and create the Symbian install 
   package. The application is installed on the device.


COMPATIBILITY 
-------------------------------------------------------------------------------

Symbian devices with Qt 4.7.4 and Qt Mobility 1.2.1 or higher, MeeGo 1.2
Harmattan, and desktop platforms with OpenGL ES 2.0 libraries installed.
  
Tested on:  
- Nokia N9
- Nokia N950
- Nokia C7-00
- Nokia N8-00
- Windows 7 

Developed with: 
- Qt SDK 1.2


LICENSE
-------------------------------------------------------------------------------

See the license text file delivered with this project. The license file is also
available online at: 
https://projects.developer.nokia.com/gles2phys/browser/trunk/Licence.txt

The Bullet physics engine is licensed under the zlib license. The license file 
of Bullet is also delivered with this project, and can be found online at: 
https://projects.developer.nokia.com/gles2phys/browser/trunk/src/bullet/BulletLicense.txt


VERSION HISTORY 
-------------------------------------------------------------------------------

1.1.1 Built and tested with Qt SDK 1.2.
1.1 Replaced ODE engine with Bullet, added Symbian support.
1.0 First version