;--------------------------------------------------
; COMMAND / RESPONSE CODES 
;--------------------------------------------------

#define kernel_cmd_get_version_info				0x01
#define kernel_cmd_CRC_area						0x02
#define kernel_cmd_read_area					0x03
#define kernel_cmd_read_programming_voltage		0x04
#define kernel_cmd_unknown_1					0x05
#define kernel_cmd_unknown_2					0x06

#define kernel_cmd_flash_enable					0x20
#define kernel_cmd_flash_disable				0x21
#define kernel_cmd_write_flash_buffer			0x22
#define kernel_cmd_validate_flash_buffer		0x23
#define kernel_cmd_commit_flash_buffer			0x24
#define kernel_cmd_blank_16k_page				0x25

#define kernel_rsp_get_version_info				0x41
#define kernel_rsp_CRC_area						0x42
#define kernel_rsp_read_area					0x43
#define kernel_rsp_read_programming_voltage		0x44
#define kernel_rsp_unknown_1					0x45
#define kernel_rsp_unknown_2					0x46

#define kernel_rsp_flash_enable					0x60
#define kernel_rsp_flash_disable				0x61
#define kernel_rsp_write_flash_buffer			0x62
#define kernel_rsp_validate_flash_buffer		0x63
#define kernel_rsp_commit_flash_buffer			0x64
#define kernel_rsp_blank_16k_page				0x65

#define kernel_rsp_error_bad_data_length		0xF0
#define kernel_rsp_error_bad_data_value			0xF1
#define kernel_rsp_error_programming_failure	0xF2
#define kernel_rsp_error_prog_voltage_low		0xF3
#define kernel_rsp_error_prog_voltage_high		0xF4
#define kernel_rsp_error_bad_crc				0xF5
#define kernel_rsp_error_bad_command			0xFF