/* Memory layout for an STM32F103RB */
MEMORY
{
   FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
   RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}
 
STACK_SIZE = 1024;
_estack = ORIGIN(RAM) + LENGTH(RAM);
_sstack = _estack - STACK_SIZE;
 
/* Output sections */
SECTIONS
{
    .text :
    {
        /* All unused bytes set to 0xFF. */
        FILL (0xff)
 
        /* ISR Vector Table first */
        _isr_vectors_offs = . - 0x08000000;
        KEEP(*(.isr_vector))
 
        . = ALIGN(4);
        *(.text)        /* Program code */
        *(.text.*)
        *(.rodata)      /* Constants */
        *(.fini)    
        *(.rodata*) 
        *(.glue_7)  
        *(.glue_7t) 
        . = ALIGN(4);
        _etext = .;
 
        _sidata = _etext;
         
    } > FLASH
 
    /* Initialized global and static variables */
    .data :
    {
        FILL (0xff)
         
        . = ALIGN(4);
        _sdata = .;     
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
         
    } > RAM AT> FLASH
 
    /* Uninitialized global and static variables */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
 
        _ebss = .;
         
    } > RAM
 
    /* Stack */
    . = _sstack;
    .stack :
    {
        . = . + STACK_SIZE;
    } > RAM
}
