#!/usr/bin/env bash

west build -b nrf52840dongle app --pristine -- "-DOVERLAY_CONFIG=overlay-bt_ll_sw_split.conf;base_48s_24s.conf"
