# XBotCore

XBotCore (Cross-Bot-Core) is a light-weight, Real-Time (RT) software platform for robotics.
It is completely open-source and is designed to be both a RT robot control framework and a software middleware. It satisfies
hard RT requirements, while ensuring 1 kHz control loop even in complex Multi-Degree-Of-Freedom systems. It provides a simple and easy-to-use middleware Application Programming Interface (API), for both RT and non-RT control frameworks. This API is completely flexible with respect to the framework a user wants to utilize. Moreover it is possible to reuse the code written using XBotCore API with different robots (cross-robot feature).

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

~~~
Giuseppe F. Rigano, Luca Muratore, Arturo Laurenzi, Enrico Mingo Hoffman, Nikos G Tsagarakis "Towards A Robot Hardware Abstraction Layer (R-HAL) Leveraging the XBot Software Framework", IEEE International Conference on Robotic Computing (IRC18), 2018.
~~~

The bibtex code for including this citation is provided:
~~~
@inproceedings{rigano_hal,
  title={Towards A Robot Hardware Abstraction Layer (R-HAL) Leveraging the XBot Software Framework},
  author={Rigano, G.F. and Muratore, L. and Laurenzi, A. and Hoffman, E.M. and Tsagarakis, N.G.},
  booktitle={IEEE International Conference on Robotic Computing, IRC18 (to appear)},
  year={2018}
}
~~~

Online documentation: https://advrhumanoids.github.io/XCM/

## Robot Hardware Abstraction Layer (R-HAL)
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
alt="OXBot R-HAL" /><br>XBot R-HAL</a>


