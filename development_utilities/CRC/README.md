# CRC
`crc.c` is a copy of the **C-Code Example for CRC Calculation** as provided by the TMC2300 datasheet (Rev 1.08 at time of writing).

## How to run
Navigate to this folder and simply run `make` from the command line terminal. Once compiled, the binary `crc` can be run from the terminal.

## Example output

```shell
user@terminal:~$ make
gcc -I.   -c -o crc.o crc.c
gcc -o ../bin/crc crc.o
user@terminal:~$ ../bin/crc
0: 0x05
1: 0x03
2: 0x06
4: 0x82 <--- CRC
```