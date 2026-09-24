# HELM_BASIC factory SD resources

This directory contains the factory files for the M0196 HELM_BASIC target.

The build creates a read-only FAT12 image from these files.
If no SD card is present, the firmware mounts that image as its file system.
A factory reset also restores an installed SD card from this image.

The initial model assigns the four primary axes to channels 1 through 4. It enables the internal CRSF module.

`SCRIPTS/TOOLS/elrs.lua` comes from the ExpressLRS version 4 source tree.

The `SCRIPTS/TOOLS/Helm.lua` tool provides the HELM dashboard, quick actions, and CRSF device parameters.
Upper-left Up opens the Helm tool from the main view.
The model maps Bind, Unbind, and VRX Scan to joystick buttons 0, 1, and 2.
