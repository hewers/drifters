/* ARM MPS2 FPGA images (AN386 = Cortex-M4, AN500 = Cortex-M7).
   Both map ZBT SRAM as code at 0x00000000 and data at 0x20000000. */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 4M
  RAM   : ORIGIN = 0x20000000, LENGTH = 4M
}
