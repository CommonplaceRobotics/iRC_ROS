# Identifier
string name
string can_id

# Device information
string hardware_ident
#string controller_type
int8 version_major
int8 version_minor

# Environmental information
float64 temperature_motor
float64 temperature_board
float64 supply_voltage
float64 motor_current

# Status
int8 error_state
string reset_state	
string motor_state	
int8 command_mode
