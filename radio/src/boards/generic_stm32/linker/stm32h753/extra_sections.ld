
/* Task stacks */
.ccm (NOLOAD) :
{
  . = ALIGN(4);
  _sccm = .;
  *(.ccm)
  . = ALIGN(4);
  _eccm = .;
} > DTCMRAM

/* Reserve stack space */
.stack (NOLOAD) :
{
  . = ALIGN(4);
  . = . + MAIN_STACK_SIZE;
  . = ALIGN(4);
} > DTCMRAM

/* Fast code */
.iram :
{
  . = ALIGN(4);
  _siram = .;
  *(.iram)
  . = ALIGN(4);
  _eiram = .;
} > ITCMRAM AT> REGION_TEXT_STORAGE

_stext_iram = LOADADDR(.iram);

/* Extra DMA section */
.dram (NOLOAD) :
{
  . = ALIGN(4);
  _s_dram = .;
  *(.dram)
  . = ALIGN(4);
  _e_dram = .;
} > RAM_D2

PROVIDE(_heap_start = _e_dram);
