#!/bin/bash

set -euxo pipefail

# remove existing blobs because otherwise this will append object files to the old blobs
rm -f bin/*.a

riscv64-unknown-elf-gcc -c -mabi=ilp32 -march=rv32imac rebase-hack.S -o bin/rebase-hack.o
ar crs bin/gd32vf103xx-hal.a bin/rebase-hack.o

rm bin/rebase-hack.o
