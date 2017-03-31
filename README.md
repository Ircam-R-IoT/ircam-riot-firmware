# R-IoT firmware v1.6

#### quick start

* Download and install Energia version 17 or lower from [energia.nu/download/](http://energia.nu/download/#previousreleases).
* Modify the `cc3200.ld` file and change the `HEAP_SIZE` value to `0x00008000` :
	* on Windows this file is located into `C:\Program Files(x86)\energia-0101E0017\hardware\cc3200\cores\cc3200`
	* on Mac OS, right-click on the Energia application and select "show package contents", then go to `Contents/Resources/Java/hardware/cc3200/cores/cc3200`
* Get the SLFS library from [github.com/Ircam-R-IoT/SLFS](https://github.com/Ircam-R-IoT/SLFS) and drop it into `Documents/Energia/libraries`.
* Open the firmware.ino file with Energia and hit the "Verify" button in the upper left corner. If it builds, you're ready to upload it to the R-IoT board.

#### going further

For more information, please visit [ismm.ircam.fr/riot/](http://ismm.ircam.fr/riot/).

#### credits

This project has been developed by the ISMM team at IRCAM-Centre Pompidou within the CoSiMa project (funded by ANR) and the MusicBricks project funded by the European Union's Horizon 2020 research and innovation programme.
