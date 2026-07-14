/* Memory map for QEMU's `mps2-an500` machine (Cortex-M7 + FPU). This is a
   throwaway QEMU-only harness binary, not a real board: sized generously so
   the linked estimator+guidance+supervisor code and the semihosting/panic
   runtime fit with headroom, not to match any real flash/RAM budget. */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 4096K
  RAM : ORIGIN = 0x20000000, LENGTH = 4096K
}
