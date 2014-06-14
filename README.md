Customized pysimiam
===================

Installation Ubuntu
-------------------

Python packages:

    $ sudo apt-get install python-matplotlib python-qt4 python-numpy

Run
---

Run the simulation:

    $ python simulation.py

Run realtime:
    
    $ python realtime.py

TODO
----
Follwoing all things that need to be done to make it more comfortable to work:

0. Create a `pysimiam_simulator` python package with only the simulator
0. Create for every robot a `pysimiam_X` python package and import needed parts
from `pysimiam_simulator`
0. Add docs 
0. Add tests, travis ci, coveralls and "read the docs" to `pysimiam_simulator`
