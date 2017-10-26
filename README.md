# R-IoT firmware v1.8

#### note

Use this version for compatibility with [MuBu](http://forumnet.ircam.fr/product/mubu-en/) v1.9.2 and later.  
You can still use v1.7 with MuBu v1.9.1 and earlier.

#### quick start

* Download and install Energia version 17 from [energia.nu/download/](http://energia.nu/download/#previousreleases).
* Modify the `cc3200.ld` file and change the `HEAP_SIZE` value to `0x00008000` :
	* on Windows this file is located into `C:\Program Files(x86)\energia-0101E0017\hardware\cc3200\cores\cc3200`
	* on Mac OS, right-click on the Energia application and select "show package contents", then go to `Contents/Resources/Java/hardware/cc3200/cores/cc3200`
* Get the SLFS library from [github.com/Ircam-R-IoT/SLFS](https://github.com/Ircam-R-IoT/SLFS) and drop it into `Documents/Energia/libraries`.
* Open the `firmware.ino` file with Energia.
* Select in menu `Tools > Boards > LauchPad W/ cc3200 (80MHz)`
* Hit the "Verify" button in the upper left corner. If it builds, you're ready to upload it to the R-IoT board.
* Choose the serial port starting with `tty`

#### going further

For more information, please visit [ismm.ircam.fr/riot/](http://ismm.ircam.fr/riot/).

#### credits

This project has been developed by the ISMM team at IRCAM-Centre Pompidou within the CoSiMa project (funded by ANR), the MusicBricks project funded by the European Union's Horizon 2020 research and innovation programme, and the RAPID-MIX project (H2020-ICT-2014-1 Project ID 644862).
