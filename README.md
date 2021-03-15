FRC2021
=======

 * Game Manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system
 * Control System:  https://docs.wpilib.org/en/stable/, https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html#new-for-2021
 * Simulation: https://www.chiefdelphi.com/t/infinite-recharge-at-home-autonav-challenges-using-wpilibs-drive-simulator/390485, https://www.chiefdelphi.com/t/ctre-simulation-discussion/390120, https://www.chiefdelphi.com/t/howto-drive-robot-in-simulator-2d-field-view/390633


Setup
-----

To work on the software, you need to install the "WPILib", which includes
Visual Studio Code, Java, and the robotics library.
See
https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/wpilib-setup.html

During the installation, you can un-check C++, the rest can remain
on default settings.

You also need 'git' from https://git-scm.com/downloads.

Once you installed it, open the 'git bash' and type this to get a copy of the source code:

```
mkdir git
cd git
git clone https://github.com/team2393/FRC2021.git
```

Then you can open `2021 WPILib VS Code` from the desktop icon
and use "File", "Open Folder" to open git/FRC2021 that you just fetched via the 'git bash'.

The above is sufficient to read and work on all the source code.

On a 'drive station' laptop that actually operates the robot you also need to install the following:

 * Un-install existing NI tools in case you have them from last year
 * Open ni-frc-2020-game-tools_20.0.1_offline.iso to install NI tools
 * Open CTRE_Phoenix_Framework_v5.19.4.1 to install Phoenix tools

Issues
======

Windows crash related to nipcibrd.sys, https://forums.ni.com/t5/FIRST-Robotics-Competition/NIPCIBRD-SYS-causes-Windows-10-Blue-Screen-of-Death/td-p/3592852
