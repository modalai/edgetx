# HELM_BASIC factory SD resources

This directory contains the SD card contents for the M0196 HELM_BASIC target.

The firmware embeds these files when `FACTORY_RESET` is enabled. A factory reset formats the SD card and restores this directory.

The initial model assigns the four primary axes to channels 1 through 4. It enables the internal CRSF module.

`SCRIPTS/TOOLS/elrs.lua` comes from the ExpressLRS version 4 source tree.
