### O.C.R.A.
#### Optimization-based Control for Robotics Applications

Give a description...

[Installation instructions](#Build & Install)

# ocra-core

Give a description...

### Code Structure
Give a description...

- core-framework
- wLQP-Control
  - wocra
  - quadprog

### core-framework

Give a description...

### wLQP-Control

Give a description...

# Installation

Give a description...

### Tested OS's

- [x] Ubuntu 12.04

So far we have only tried OCRA on Ubuntu 12.04 but in theory any linux distro should work if the dependencies are met and don't conflict with any system libs/headers. If you manage to build, install and use OCRA in any other platform please let us know and we can add it to the list with any helpful notes you provide along with it.

### Dependencies

`ocra-core` depends on Boost along with Eigen 3.0 and its unsupported template library LGSM. If you are in linux and have `apt-get` you can install via:
```bash
sudo apt-get install libboost-dev libeigen3-dev
```
Please make sure that the Eigen version installed is <=3.0.5 otherwise you will not be able to build the ocra-core libs. Once you have Eigen installed we have to make sure that PkgConfig will be able to find the "unsupported" headers. To do this, we have to modify the eigen3 package configuration file.

```bash
sudo nano /usr/share/pkgconfig/eigen3.pc
```
You should now be inside of the nano text editor. At the bottom of the file the last line should read:
```pc
Cflags: -I/usr/include/eigen3 -I/usr/include/eigen3/unsupported
```
if it only reads:
```pc
Cflags: -I/usr/include/eigen3
```
please add `-I/usr/include/eigen3/unsupported` as above. Now that the Cflags are correct, press `ctrl + x`, answer `Y` to accept the changes and press `enter` or `return`.

That is it for the dependencies! Hooray!

#### Some notes on Eigen 3.2

Unfortunately `ocra-core` is incompatible with the current version, 3.2+, of Eigen. Specifically, the biggest issue is the use of `<Ref>` for passing Eigen objects as function arguments. Usage of `<Ref>` in any code linking to the `ocra-core` libs will break your build. As you can imagine this is somewhat of a sore point for us since we can't use all of the neat toys in Eigen 3.2+ but we are working on upgrading `ocra-core`. However, this process will be long so have patience grasshopper.  

### Build & Install
**WARNING**
*This is an experimental set of libs and there are no guarantees that they will not do damage to your computer. We take no responsibility for what happens if you install them. That being said, if you follow these instructions you should be fine.*

Okay that's out of the way... phew!

So you have your dependencies installed and your package config files ready to go. Let's run through the whole process step by step. We start in the home directory of user, "bob": `/home/bob`.

First we clone the repo.
```bash
git clone https://github.com/ocra-recipes/ocra-core.git
cd ocra-core/
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/bob/ocra-install-dir ..
```
The install prefix, `-DCMAKE_INSTALL_PREFIX=/home/bob/ocra-install-dir`, is an optional Cmake argument that allows you to specify where the built libs, headers and config files will be installed on your system. Because these are experimental libs, our suggestion is to specify a directory under `/home/{your user name}/{some installation directory}` (remember to leave out the brackets). This will avoid any potential conflicts with system libs and allow you to easily "uninstall" (delete) everything if you need to.

**NOTE:** If you do not set the install prefix then the default `/usr/local/` will be used.

Okay, let's build/install the darn thing already!
```bash
make install
```
Since we didn't install it to `/usr/local/`, we need to tell package config where to look for ocra.

```bash
nano ~/.bashrc
```
We are now in the nano text editor. Scroll down to the bottom and write the following line:
```
export OCRA_INSTALL=/home/bob/ocra-install-dir
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:${OCRA_INSTALL}/lib/pkgconfig
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${OCRA_INSTALL}/lib
```
Rememeber unless your name is bob, make sure to change the user name (and any other directory names) in the path. Press `ctrl + x`, answer `Y` to accept the changes and `enter`.

Now we have to source the .bashrc file to update the environment variables.
```bash
source ~/.bashrc
```
That's it! The core framework is now built, installed and ready to rock!


### Enjoying your OCRA...
Well now that you have `ocra-core` up and running, you probably want to try it out n'est pas? Well mosey on over to the [ocra-wbi-plugins](https://github.com/ocra-recipes/ocra-wbi-plugins) repo and follow the instructions.

Want to contribute? Maybe build a plugin or two? Read the [Contributing  section](#Contributing) for details on how to interface with OCRA and use it for world domination.

### Contributing

Give a description...
