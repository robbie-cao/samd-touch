# Qtouch on SAMD20/21 and QT6 Xplained

## How to Build with `arm-none-eabi-` Toolchain in Cygwin/Linux/MacOS

### Build in App Dir

```
$ cd project_root_dir
$ source envsetup.sh    (*MUST*, one time shot)
$ cd apps/hmc
$ make clean
$ make
- or -
$ make -f Makefile
```

### Build App from Top

```
$ cd project_root_dir
$ source envsetup.sh    (*MUST*, one time shot)
$ mmm apps/hmc

Usage:
  mmm [path] [option]
  - path   - relative path to Makefile (for an App) to be built with
  - option - target to make, eg clean, debug, release, etc
Example:
  mmm apps/hmc clean (* at top folder)
  mmm apps/hmc       (* at top folder)
  mmm . clean        (* at apps/hmc)
  mmm                (* at apps/hmc)
```


## How to Flash Image

### Flash by Path From Top

```
$ cd project_root_dir
$ ./fam.sh [path] [action]

Usage:
fam.sh [path] [option]
* path:
    - App path, relative path to exe/xxx.bin in unix format,
      eg, MUA/main (exe not included in path)
* option:
    - E - erase
    - P - program (default)
	- R - read
    - EP
```

Example:

```
$ ./fam.sh apps/hmc
$ ./fam.sh apps/hmc EP
$ ./fam.sh . E
$ ./fam.sh
$ ./fam.sh . P
```

<span style="color:red">
**Support Linux and Mac OS.**
</span>

### Flash Script Wrapper

```
$ cd project_root_dir
$ source envsetup.sh
$ fa [path] [action]
-or-
$ cd <path_to_app>
$ fa
```

<span style="color:red">
**Support Linux and Mac OS.**

**`fa` is a wrapper of `fam.sh`, it use the same parameters and can be used anywhere which have exe/xxx.bin in the folder without parameters.**
</span>



### `edbg` Usage

```
Usage: edbg [options]
Options:
  -h, --help                 print this help message and exit
  -b, --verbose              print verbose messages
  -e, --erase                perform a chip erase before programming
  -p, --program              program the chip
  -v, --verify               verify memory
  -k, --lock                 lock the chip (set security bit)
  -r, --read                 read the whole content of the chip flash
  -f, --file <file>          binary file to be programmed or verified
  -t, --target <name>        specify a target type (use '-t list' for a list of supported target types)
  -l, --list                 list all available debuggers
  -s, --serial <number>      use a debugger with a specified serial number
  -o, --offset <number>      offset for the operation
  -z, --size <number>        size for the operation
```

Example

```
$ edbg -bpv -t atmel_cm0p -f apps/hmc/samd_qtouch_mutlcap_example_flash.bin
Debugger: ATMEL EDBG CMSIS-DAP ATML2407060200000332 02.01.0157 (S)
Target type: Cortex-M0+
Target: SAM D21J18
Programming....... done.
Verification....... done.
```

