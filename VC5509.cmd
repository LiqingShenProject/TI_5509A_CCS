-c
-stack 800
-sysstack 800


MEMORY
{
/*-----------system--------------------------------------*/
    DARAM1:	o=0x100,	l=0x7f00	/*16k word  32k byte*/
    VECT :  o=0x8000,	l=0x100
    DARAM2: o=0x8100,	l=0x7f00	/*16k word  32k byte*/



/*-----------user--------------------------------------*/
    SARAM1:  o=0x10000,	l=0x10000	/*32k word   64k byte : boxdata*/
    SARAM2:  o=0x20000,	l=0x8000	/*16k word   32k byte : user_temp*/
    SARAM3:  o=0x28000,	l=0x8000	/*16k word   32k byte : user_temp*/
    SARAM4:  o=0x30000,	l=0x10000	/*32k word   64k byte : Text code*/

}

// section mapping
SECTIONS
{

    .vectors: {} > VECT
    .cinit:   {} > DARAM1
    .sysinit: {} > DARAM1
    .bss:     {} > DARAM1
    .far:     {} > DARAM1
    .const:   {} > DARAM1
    .switch:  {} > DARAM1
    .sysmem:  {} > DARAM1
    .cio:     {} > DARAM1
    .csldata  {} > DARAM1

    .FLASHDATA{} > DARAM2

    .BOXDATA  {} > SARAM1

    .USER:    {} > SARAM2

	.text:    {} > SARAM4

	.MarkDATA {} > SDRAM
}
/*************************************************
**.text : code segement**
**.dtat : initial data segment**
**.bss  : dynamic data segment**
**.stack : system stack**
**.cio   : printf & related**
**.sysmem : marco segment**
** "vectors" interrupt vector£¨.sect "vectors"£©**
**************************************************/
