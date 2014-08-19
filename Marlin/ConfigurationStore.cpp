#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "V12"

#ifdef EEPROM_SETTINGS
void Config_StoreSettings() 
{
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first 
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);  
  EEPROM_WRITE_VAR(i,max_feedrate);  
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  EEPROM_WRITE_VAR(i,add_homeing);
  #ifdef DELTA
  EEPROM_WRITE_VAR(i,endstop_adj);
  #endif
  #ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,bed_level_probe_offset[0]);
  EEPROM_WRITE_VAR(i,bed_level_probe_offset[1]);
  EEPROM_WRITE_VAR(i,bed_level_probe_offset[2]);
  #ifdef PIDTEMP
    EEPROM_WRITE_VAR(i,Kp);
    EEPROM_WRITE_VAR(i,Ki);
    EEPROM_WRITE_VAR(i,Kd);
  #else
		float dummy = 3000.0f;
    EEPROM_WRITE_VAR(i,dummy);
		dummy = 0.0f;
    EEPROM_WRITE_VAR(i,dummy);
    EEPROM_WRITE_VAR(i,dummy);
  #endif
  #ifndef DOGLCD
    int lcd_contrast = 32;
  #endif
  EEPROM_WRITE_VAR(i,lcd_contrast);
  EEPROM_WRITE_VAR(i,base_min_pos[0]);
  EEPROM_WRITE_VAR(i,base_max_pos[0]);
  EEPROM_WRITE_VAR(i,base_min_pos[1]);
  EEPROM_WRITE_VAR(i,base_max_pos[1]);
  EEPROM_WRITE_VAR(i,base_min_pos[2]);
  EEPROM_WRITE_VAR(i,base_max_pos[2]);
  EEPROM_WRITE_VAR(i,home_dir[0]);
  EEPROM_WRITE_VAR(i,home_dir[1]);
  EEPROM_WRITE_VAR(i,home_dir[2]);
  EEPROM_WRITE_VAR(i,reverse_motor[0]);
  EEPROM_WRITE_VAR(i,reverse_motor[1]);
  EEPROM_WRITE_VAR(i,reverse_motor[2]);
  EEPROM_WRITE_VAR(i,reverse_motor[3]);
  EEPROM_WRITE_VAR(i,enable_auto_bed_leveling);
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifndef DISABLE_M503
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
    SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
    SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
    SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
    SERIAL_ECHOLN("");
      
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
    SERIAL_ECHOPAIR(" Y",max_feedrate[1] ); 
    SERIAL_ECHOPAIR(" Z", max_feedrate[2] ); 
    SERIAL_ECHOPAIR(" E", max_feedrate[3]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] ); 
    SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] ); 
    SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
    SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M204 S",acceleration ); 
    SERIAL_ECHOPAIR(" T" ,retract_acceleration);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
    SERIAL_ECHOPAIR(" T" ,mintravelfeedrate ); 
    SERIAL_ECHOPAIR(" B" ,minsegmenttime ); 
    SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
    SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
    SERIAL_ECHOPAIR(" E" ,max_e_jerk);
    SERIAL_ECHOLN(""); 

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Home offset (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M206 X",add_homeing[0] );
    SERIAL_ECHOPAIR(" Y" ,add_homeing[1] );
    SERIAL_ECHOPAIR(" Z" ,add_homeing[2] );
    SERIAL_ECHOLN("");
#ifdef DELTA
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Endstop adjustement (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M666 X",endstop_adj[0] );
    SERIAL_ECHOPAIR(" Y" ,endstop_adj[1] );
    SERIAL_ECHOPAIR(" Z" ,endstop_adj[2] );
    SERIAL_ECHOLN("");
#endif
#ifdef PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("PID settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M301 P",Kp); 
    SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki)); 
    SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd));
    SERIAL_ECHOLN(""); 
#endif
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Min position (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M210 X" , base_min_pos[0] );
    SERIAL_ECHOPAIR(" Y" , base_min_pos[1] );
    SERIAL_ECHOPAIR(" Z" , base_min_pos[2] );
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Max position (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M211 X" , base_max_pos[0] );
    SERIAL_ECHOPAIR(" Y" , base_max_pos[1] );
    SERIAL_ECHOPAIR(" Z" , base_max_pos[2] );
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Bed probe offset (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M212 X" , bed_level_probe_offset[0] );
    SERIAL_ECHOPAIR(" Y" , bed_level_probe_offset[1] );
    SERIAL_ECHOPAIR(" Z" , bed_level_probe_offset[2] );
    SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Endstop seek direction:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M504 X" ,(float)home_dir[0]);
	SERIAL_ECHOPAIR(" Y" ,(float)home_dir[1]);
	SERIAL_ECHOPAIR(" Z" ,(float)home_dir[2]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Motors reversed:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M505 X" ,(long unsigned int)reverse_motor[0]);
	SERIAL_ECHOPAIR(" Y" ,(long unsigned int)reverse_motor[1]);
	SERIAL_ECHOPAIR(" Z" ,(long unsigned int)reverse_motor[2]);
	SERIAL_ECHOPAIR(" E" ,(long unsigned int)reverse_motor[3]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Auto leveling:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M506 ",(long unsigned int) enable_auto_bed_leveling);
	SERIAL_ECHOLN("");
} 
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);  
        EEPROM_READ_VAR(i,max_feedrate);  
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
        
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
        
        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,minsegmenttime);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        EEPROM_READ_VAR(i,add_homeing);
        #ifdef DELTA
        EEPROM_READ_VAR(i,endstop_adj);
        #endif
        #ifndef ULTIPANEL
        int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
        int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
        #endif
        EEPROM_READ_VAR(i,plaPreheatHotendTemp);
        EEPROM_READ_VAR(i,plaPreheatHPBTemp);
        EEPROM_READ_VAR(i,plaPreheatFanSpeed);
        EEPROM_READ_VAR(i,absPreheatHotendTemp);
        EEPROM_READ_VAR(i,absPreheatHPBTemp);
        EEPROM_READ_VAR(i,absPreheatFanSpeed);
        EEPROM_READ_VAR(i,bed_level_probe_offset[0]);
        EEPROM_READ_VAR(i,bed_level_probe_offset[1]);
        EEPROM_READ_VAR(i,bed_level_probe_offset[2]);
        #ifndef PIDTEMP
        float Kp,Ki,Kd;
        #endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);
        #ifndef DOGLCD
        int lcd_contrast;
        #endif
        EEPROM_READ_VAR(i,lcd_contrast);
        EEPROM_READ_VAR(i,base_min_pos[0]);
        EEPROM_READ_VAR(i,base_max_pos[0]);
        EEPROM_READ_VAR(i,base_min_pos[1]);
        EEPROM_READ_VAR(i,base_max_pos[1]);
        EEPROM_READ_VAR(i,base_min_pos[2]);
        EEPROM_READ_VAR(i,base_max_pos[2]);
		EEPROM_READ_VAR(i,home_dir[0]);
		EEPROM_READ_VAR(i,home_dir[1]);
		EEPROM_READ_VAR(i,home_dir[2]);
		EEPROM_READ_VAR(i,reverse_motor[0]);
		EEPROM_READ_VAR(i,reverse_motor[1]);
		EEPROM_READ_VAR(i,reverse_motor[2]);
		EEPROM_READ_VAR(i,reverse_motor[3]);
		EEPROM_READ_VAR(i,enable_auto_bed_leveling);
		
		update_home_direction(); // updates parameters related to motor direction / home direction which were previously defined in preprocessor
		
		// Call updatePID (similar to when we have processed M301)
		updatePID();
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
    }
    else
    {
        Config_ResetDefault();
    }
    #ifdef EEPROM_CHITCHAT
      Config_PrintSettings();
    #endif
}
#endif

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (short i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }
    
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
    
    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
#ifdef DELTA
    endstop_adj[0] = endstop_adj[1] = endstop_adj[2] = 0;
#endif
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
    bed_level_probe_offset[0] = X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT;
    bed_level_probe_offset[1] = Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT;
    bed_level_probe_offset[2] = Z_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT;
#ifdef DOGLCD
    lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

	base_min_pos[0] = X_MIN_POS_DEFAULT;
	base_min_pos[1] = Y_MIN_POS_DEFAULT;
	base_min_pos[2] = Z_MIN_POS_DEFAULT;
	base_max_pos[0] = X_MAX_POS_DEFAULT;
	base_max_pos[1] = Y_MAX_POS_DEFAULT;
	base_max_pos[2] = Z_MAX_POS_DEFAULT;
	home_dir[X_AXIS] = X_HOME_DEFAULT;
	home_dir[Y_AXIS] = Y_HOME_DEFAULT;
	home_dir[Z_AXIS] = Z_HOME_DEFAULT;
	reverse_motor[X_AXIS] = X_MOTOR_REVERSE_DEFAULT;
	reverse_motor[Y_AXIS] = Y_MOTOR_REVERSE_DEFAULT;
	reverse_motor[Z_AXIS] = Z_MOTOR_REVERSE_DEFAULT;
	reverse_motor[E_AXIS] = E_MOTOR_REVERSE_DEFAULT;
	enable_auto_bed_leveling = ENABLE_AUTO_BED_LEVELING_DEFAULT;
	
	update_home_direction(); // updates parameters related to motor direction / home direction which were previously defined in preprocessor

	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
}

void update_home_direction(void)
{
	#ifdef MANUAL_HOME_POSITIONS  // Use manual limit switch locations
	home_pos[X_AXIS] = MANUAL_X_HOME_POS;
	home_pos[Y_AXIS] = MANUAL_Y_HOME_POS;
	home_pos[Z_AXIS] = MANUAL_Z_HOME_POS;
	#else //Set min/max homing switch positions based upon homing direction and min/max travel limits
	//X axis
	if(home_dir[X_AXIS]== -1)
	{
		#ifdef BED_CENTER_AT_0_0
		home_pos[X_AXIS] = X_MAX_LENGTH * -0.5;
		#else
		home_pos[X_AXIS] = base_min_pos[X_AXIS];
		#endif //BED_CENTER_AT_0_0
		min_pin[X_AXIS] = X_STOP_PIN;
		max_pin[X_AXIS] = -1;
	}
	else
	{
		#ifdef BED_CENTER_AT_0_0
		home_pos[X_AXIS] = X_MAX_LENGTH * 0.5;
		#else
		home_pos[X_AXIS] = base_max_pos[X_AXIS];
		#endif //BED_CENTER_AT_0_0
		min_pin[X_AXIS] = -1;
		max_pin[X_AXIS] = X_STOP_PIN;
	}
	//Y axis
	if(home_dir[Y_AXIS]== -1)
	{
		#ifdef BED_CENTER_AT_0_0
		home_pos[Y_AXIS] = Y_MAX_LENGTH * -0.5;
		#else
		home_pos[Y_AXIS] = base_min_pos[Y_AXIS];
		#endif //BED_CENTER_AT_0_0
		min_pin[Y_AXIS] = Y_STOP_PIN;
		max_pin[Y_AXIS] = -1;
	}
	else
	{
		#ifdef BED_CENTER_AT_0_0
		home_pos[Y_AXIS] = Y_MAX_LENGTH * 0.5;
		#else
		home_pos[Y_AXIS] = base_max_pos[Y_AXIS];
		#endif //BED_CENTER_AT_0_0
		min_pin[Y_AXIS] = -1;
		max_pin[Y_AXIS] = Y_STOP_PIN;
	}
	// Z axis
	if(home_dir[Z_AXIS]== -1){
		home_pos[Z_AXIS] = base_min_pos[Z_AXIS];
		min_pin[Z_AXIS] = Z_STOP_PIN;
		max_pin[Z_AXIS] = -1;
	}
	else{
		home_pos[Z_AXIS] = base_max_pos[Z_AXIS];
		min_pin[Z_AXIS] = -1;
		max_pin[Z_AXIS] = Z_STOP_PIN;
	}
	#endif //End auto min/max positions
}