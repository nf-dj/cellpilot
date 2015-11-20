# CellPilot

A controller to pilot UAV/drones via 3G/4G cellular connection.

WARNING: this version of CellPilot will soon be replaced by a new controller using TinyCom instead of Raspberry Pi (http://github.com/nfco/tinycom).

## Pictures

A view of the PCB, a Raspberry Pi "hat":

![CellPilot board](https://raw.githubusercontent.com/nfco/cellpilot/master/img/cellpilot_board.jpg)

Inside a 3d-printed prototype box:

![CellPilot set](https://raw.githubusercontent.com/nfco/cellpilot/master/img/cellpilot_set.jpg)

CellPilot installed on a DIY drone:

![CellPilot drone](https://raw.githubusercontent.com/nfco/cellpilot/master/img/cellpilot_drone.jpg)

We still rely on an external controller for stabilization.
But a 10DOF sensor module can be installed on the CellPilot board and we can later try to do the stabilization directly on the STM32 processor of the board. (a prototype firmware to do this is already in the repository)

## Video

CellPilot test flight (64Kbps, 10fps):

[![CellPilot test flight](http://img.youtube.com/vi/GPAqMF_AkHQ/0.jpg)](http://www.youtube.com/watch?v=GPAqMF_AkHQ)

(the strange colors in the video are due to the PiNoir camera, we will make a new video with a regular Pi camera and some vibration dampening)

## Ordering

A 3d-printed plastic case can be ordered here:
https://www.shapeways.com/shops/netforce

![CellPilot case](https://images1.sw-cdn.net/model/picture/625x465_3887793_12400456_1444480401.jpg)

## Contact

Send email to dj@netforce.com.
