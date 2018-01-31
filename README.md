# XBotCore

XBotCore is the new software architecture to control ADVR robots: it uses Xenomai API to satisfy Real-Time requirements. 

Moreover it provides XDDP pipes communication with a Not-Real-Time communication API. 

## Reference paper
A paper describing the XBotCore software architecture can be downloaded [here](https://www.researchgate.net/publication/316514802_XBotCore_A_Real-Time_Cross-Robot_Software_Platform) . If you're going to use this library for your work, please quote it within any resulting publication:
~~~
Luca Muratore, Arturo Laurenzi, Enrico Mingo Hoffman, Alessio Rocchi, Darwin G Caldwell, Nikos G Tsagarakis "XBotCore: A Real-Time Cross-Robot Software Platform", IEEE International Conference on Robotic Computing (IRC17), 2017.
~~~

The bibtex code for including this citation is provided:
~~~
@inproceedings{muratore2017xbotcore,
  title={XBotCore: A Real-Time Cross-Robot Software Platform},
  author={Muratore, Luca and Laurenzi, Arturo and Hoffman, Enrico Mingo and Rocchi, Alessio and Caldwell, Darwin G and Tsagarakis, Nikos G},
  booktitle={IEEE International Conference on Robotic Computing, IRC17},
  year={2017}
}
~~~

Online documentation: https://advrhumanoids.github.io/XCM/

## Robotic Hardware Abstracion Layer
We present a new Robot Hardware
Abstraction Layer (R-HAL) that permits to seamlessly program
and control any robotic platform powered by the XBot control
software framework. The R-HAL is extensively validated through
simulation trials and experiments with a wide range of dissimilar
robotic platforms, among them the COMAN and WALKMAN
humanoids, the KUKA LWR and the CENTAURO upper
body. The results attained demonstrate in practise the gained
benefits in terms of code compatibility, reuse and portability,
and finally unified application programming even for robots
with significantly diverse hardware. Furthermore, it is shown
that the implementation and integration of the R-HAL within
the XBot framework does not generate additional computational
overheads for the robot computational units.

<a href="https://www.youtube.com/watch?v=lcAB4lHbma0
" target="_blank"><img src="http://i3.ytimg.com/vi/lcAB4lHbma0/maxresdefault.jpg" 
alt="OpenSoT + PI" width="480" height="360" border="10" /><br>XBot R-HAL</a>


