# gr-csitool
gr-csitool is a customized gnuradio tool for sniffering 802.11 g packets and reporting CSI. It is compatible to gnuradio V3.7.

To build gr-csitool, you have to install two modules, gr-csitool and gr-wifirecv. 

In the root directory of each module, run, 

~~~
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig
~~~

To further obtain MAC data, you have to install [gr-ieee802-11](https://github.com/bastibl/gr-ieee802-11).

Examples (Wi-Fi receiver) are provided in the project.
