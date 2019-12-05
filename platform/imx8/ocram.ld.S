SECTIONS {
    . = %SRAM_VBASE%;
    .ocram_data (NOLOAD): {
        __ocram_data_start = .;
        KEEP (*(.ocram.data))
        __ocram_data_end = .;
    }
}
INSERT AFTER .dummy_end_of_ram
