FRC2021
=======

 * Game Manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system
 * Control System:  https://docs.wpilib.org/en/stable/, https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html#new-for-2021
 * Simulation: https://www.chiefdelphi.com/t/infinite-recharge-at-home-autonav-challenges-using-wpilibs-drive-simulator/390485, https://www.chiefdelphi.com/t/ctre-simulation-discussion/390120


Setup
-----

 * Optionally un-install existing NI tools
 * Open ni-frc-2020-game-tools_20.0.1_offline.iso to install NI tools
 * Open WPILib_Windows64-2021.2.1 to install WPILib (un-checked C++)
 * Open CTRE_Phoenix_Framework_v5.19.4.1 to install Phoenix tools
 
Import of 2020 Sources
----------------------

Simply opening the existing "Folder" in VSCode triggered an 'import' dialog.
Imported into a foler FRC2021.
Build failed because of missing dependencies:

 * New Command lib and Phoenix could be added via Command, "WPILib: Manage Vendor Libs", Install offline
 * vendordeps/REVColorSensorV3.json was copied from old sources
 
 Result compiles and deploys.
