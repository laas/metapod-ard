=======
metapod-ard
===========

Abstract Robot Dynamics implementation with metapod

This package implements the adaptor design between the metapod
package available at:
https://github.com/laas/metapod.git

and the abstract-robot-dynamics package available at:

https://github.com/laas/abstract-robot-dynamics.git

The aim of the abstract robot dynamics is to 
separate the handling of robot dynamic model and the underlying implementation.

Therefore this package is only aiming at providing a way to generate a dynamic
library for various robots being used in a software platforms relying
on abstract-robot-dynamics.
It should NOT be use to allow the user to directly access
the underlying structure.
Otherwise you should use directly metapod.

Coding style
------------
Please follow strictly the C++ coding style 
http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml


Copyright 2013, LAAS/CNRS, Olivier STASSE

