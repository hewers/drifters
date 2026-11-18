/* STM32H743ZI. Stack and data go in DTCM: zero wait states, not cached, and
   readable by the probe while the core runs, which RTT relies on. Code runs
   from flash through the instruction cache, as production firmware would. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2M
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
