### Zorro-Blue configuration
copy the contents of the releases to the zorro-blue sd-card over USB
[releases](https://gitlab.com/voxl-public/voxl-sdk/accessories/edgetx-sdcard/-/releases) contains the zorro-blue configuration


To create a new release
1. increment sdcard.version.x.x.x, then create a tag with the same version
2. increment the current tag eg `git tag x.x.x` then push it with `git push --tags`

CI will automatically create a new release and create a changelog based on the previous commit messages. Please be descriptive when updating this repo.

