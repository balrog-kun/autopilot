MEMORY
{
	flash (rx) : ORIGIN = 0x0, LENGTH = 0x8000
	ram (rwx) : ORIGIN = 0x10000000, LENGTH = 0x2000
}

ENTRY(start)

SECTIONS
{
	.text :
	{
		KEEP(*(.vector_table))
		*(.text*)
		*(.rodata*)
	} > flash
	_etext = .;

	.data :
	{
		_data = .;
		*(.data*)
		_edata = .;
	} > ram AT>flash

	.bss :
	{
		_bss = .;
		*(.bss*)
		_ebss = .;
	} > ram

	_stack_top = ORIGIN(ram) + LENGTH(ram);
}
