/*
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file cctalk_constants.h
 *
 * ESP-IDF library for communicating with cctalk devices via UART
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef CCTALK_CONSTANTS_H
#define CCTALK_CONSTANTS_H

// just the definitions for cctalk headers
#define CCTALK_RESET_DEVICE 1
#define CCTALK_REQUEST_COMMS_STATUS_VARIABLES 2
#define CCTALK_CLEAR_COMMS_STATUS_VARIABLES 3
#define CCTALK_REQUEST_COMMS_REVISION 4
#define CCTALK_REQUEST_SERVICE_STATUS 104
#define CCTALK_DATA_STREAM 105
#define CCTALK_REQUEST_ESCROW_STATUS 106
#define CCTALK_OPERATE_ESCROW 107
#define CCTALK_REQUEST_ENCRYPTED_MONETARY_ID 108
#define CCTALK_REQUEST_ENCRYPTED_HOPPER_STATUS 109
#define CCTALK_SWITCH_ENCRYPTION_KEY 110
#define CCTALK_REQUEST_ENCRYPTION_SUPPORT 111
#define CCTALK_REQUEST_ENCRYPTED_EVENTS 112
#define CCTALK_SWITCH_BAUD_RATE 113
#define CCTALK_REQUEST_USB_ID 114
#define CCTALK_REQUEST_REAL_TIME_CLOCK 115
#define CCTALK_MODIFY_REAL_TIME_CLOCK 116
#define CCTALK_REQUEST_CASHBOX_VALUE 117
#define CCTALK_MODIFY_CASHBOX_VALUE 118
#define CCTALK_REQUEST_HOPPER_BALANCE 119
#define CCTALK_MODIFY_HOPPER_BALANCE 120
#define CCTALK_PURGE_HOPPER 121
#define CCTALK_REQUEST_ERROR_STATUS 122
#define CCTALK_REQUEST_ACTIVITY_REGISTER 123
#define CCTALK_VERIFY_MONEY_OUT 124
#define CCTALK_PAY_MONEY_OUT 125
#define CCTALK_CLEAR_MONEY_COUNTER 126
#define CCTALK_REQUEST_MONEY_OUT 127
#define CCTALK_REQUEST_MONEY_IN 128
#define CCTALK_READ_BARCODE_DATA 129
#define CCTALK_REQUEST_INDEXED_HOPPER_DISPENSE_COUNT 130
#define CCTALK_REQUEST_HOPPER_COIN_VALUE 131
#define CCTALK_EMERGENCY_STOP_VALUE 132
#define request_hopper_polling_value 133
#define dispense_hopper_value 134
#define set_accept_limit 135
#define store_encryption_code 136
#define switch_encryption_code 137
#define finish_firmware_upgrade 138
#define begin_firmware_upgrade 139
#define upload_firmware 140
#define request_firmware_upgrade_capability 141
#define finish_bill_table_upgrade 142
#define begin_bill_table_upgrade 143
#define upload_bill_tables 144
#define request_currency_revision 145
#define operate_bidirectional_motors 146
#define perform_stacker_cycle 147
#define read_opto_voltages 148
#define request_individual_error_counter 149
#define request_individual_accept_counter 150
#define test_lamps 151
#define request_bill_operating_mode 152
#define modify_bill_operating_mode 153
#define route_bill 154
#define request_bill_position 155
#define request_country_scaling_factor 156
#define request_bill_id 157
#define modify_bill_id 158
#define read_buffered_bill_events 159
#define request_cipher_key 160
#define pump_rng 161
#define modify_inhibit_and_override_registers 162
#define test_hopper 163
#define enable_hopper 164
#define modify_variable_set 165
#define request_hopper_status 166
#define dispense_hopper_coins 167
#define request_hopper_dispense_count 168
#define request_address_mode 169
#define request_base_year 170
#define request_hopper_coin 171
#define emergency_stop 172
#define request_thermistor_reading 173
#define request_payout_float 174
#define modify_payout_float 175
#define request_alarm_counter 176
#define handheld_function 177
#define request_bank_select 178
#define modify_bank_select 179
#define request_security_setting 180
#define modify_security_setting 181
#define download_calibration_info 182
#define upload_window_data 183
#define request_coin_id 184
#define modify_coin_id 185
#define request_payout_capacity 186
#define modify_payout_capacity 187
#define request_default_sorter_path 188
#define modify_default_sorter_path 189
#define keypad_control 191
#define request_build_code 192
#define request_fraud_counter 193
#define request_reject_counter 194
#define request_last_modification_date 195
#define request_creation_date 196
#define calculate_rom_checksum 197
#define counters_to_eeprom 198
#define configuration_to_eeprom 199
#define acmi_unencrypted_product_id 200
#define request_teach_status 201
#define teach_mode_control 202
#define display_control 203
#define meter_control 204
#define request_payout_absolute_count 207
#define modify_payout_absolute_count 208
#define request_sorter_paths 209
#define modify_sorter_paths 210
#define power_management_control 211
#define request_coin_position 212
#define request_option_flags 213
#define write_data_block 214
#define read_data_block 215
#define request_data_storage_availability 216
#define request_payout_highlow_status 217
#define enter_pin_number 218
#define enter_new_pin_number 219
#define acmi_encrypted_data 220
#define request_sorter_override_status 221
#define modify_sorter_override_status 222
#define modify_encrypted_inhibit_and_override_registers 223
#define request_encrypted_product_id 224
#define request_accept_counter 225
#define request_insertion_counter 226
#define request_master_inhibit_status 227
#define modify_master_inhibit_status 228
#define read_buffered_credit_or_error_codes 229
#define request_inhibit_status 230
#define modify_inhibit_status 231
#define perform_self_check 232
#define latch_output_lines 233
#define send_dh_public_key 234
#define read_dh_public_key 235
#define read_opto_states 236
#define read_input_lines 237
#define test_output_lines 238
#define CCTALK_OPERATE_MOTORS 239
#define CCTALK_TEST_SOLENOIDS 240
#define request_software_revision 241
#define request_serial_number 242
#define request_database_version 243
#define request_product_code 244
#define request_equipment_category_id 245
#define request_manufacturer_id 246
#define request_variable_set 247
#define request_status 248
#define request_polling_priority 249
#define CCTALK_ADDRESS_RANDOM 250
#define CCTALK_ADDRESS_CHANGE 251
#define CCTALK_ADDRESS_CLASH 252
#define CCTALK_ADDRESS_POLL 253
#define CCTALK_SIMPLE_POLL 254
#define CCTALK_FACTORY_SETUP_AND_TEST 255



#endif /* CCTALK_CONSTANTS_H */
