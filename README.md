# Broadcast Audio Multi Source
This application can be built in different configurations to create continuous loop Broadcast Audio Sources, using pre-encoded sine wave audio of different frequency, stored with the application in flash.

The application is a slightly modified version of the [BAP Broadcast Source sample](https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/bluetooth/bap_broadcast_source).


# Getting started...
If you haven't done it yet, first go to [The Zephyr getting started guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) and install all dependencies (I'd recommend following the path with the virtual python environment).

# For development
For developers of the application, first do a fork of the repo.  Then do the following:

Make a local workspace folder (to hold the repo, zephyr and west modules):

```
mkdir my-workspace
cd my-workspace
```

Clone the repo:

```
git clone git@github.com:<your handle>/multi-source.git
```

Initialize west and update dependencies:

```
west init -l multi-source
west update
```

# For normal use (non-development)
This repo contains a stand alone Zephyr application that can be fetched and initialized like this:

```
west init -m https://github.com/astraeuslabs/multi-source --mr main my-workspace
```

Then use west to fetch dependencies:

```
cd my-workspace
west update
```

# Build and flash

Go to the repo folder:

```
cd multi-source
```

## nRF52840 Dongle
There are a few scripts for building and flashing the nRF52840 Dongle - run them in this order (example for 24S 16S config):

```shell
compile_app_24S_16S.sh
create_flash_package.sh
flash_dongle.sh
```

Note: You'll need to get the dongle in DFU mode by pressing the small side buttton with a nail. The dongle should then start fading a red light in and out.
