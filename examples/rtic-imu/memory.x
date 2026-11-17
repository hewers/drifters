/* STM32F405RG: 1 MiB of flash and 128 KiB of SRAM (the 64 KiB of CCM is not
   used). QEMU's netduinoplus2 machine maps both where the part does. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
