#include "pros/apix.h"
#define LOG_LEVEL_FILE LOG_LEVEL_DEBUG
#include "pal/log.h"
#include "pal/gterm.h"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	log_init();
	gterm_init(NULL);
	LOG_ALWAYS("In Initialize");
	gterm_print("In Initialize");

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	/* Find our motor port and sensor port */
	int motor_port = -1;
	int sensor_port = -1;

	for(int i = 0; i < 21; i++)
	{
		v5_device_e_t type = registry_get_plugged_type(i);
		LOG_DEBUG("Port %02d has device class %03d",(i+1),type);

		/* If it's a motor, set it to motor port */
		if(E_DEVICE_MOTOR == type)
		{
			motor_port = i+1; /* i is zero indexed, need to be 1 indexed */
		}
		else if(E_DEVICE_ROTATION == type)
		{
			sensor_port = i+1;
		}
	}

	if(motor_port > 0 && sensor_port > 0)
	{
		LOG_ALWAYS("Using motor %d and sensor %d",motor_port,sensor_port);
		gterm_print("Using motor %d and sensor %d",motor_port,sensor_port);
	}
	else if(motor_port > 0)
	{
		LOG_ERROR("Found motor %d but could not find sensor, please connect and restart",motor_port);
		gterm_print("Found motor %d but #ff0000 could not find sensor#",motor_port);
		gterm_print("#ff8000 Please connect sensor and restart program#");
		return;
	}
	else if(sensor_port > 0)
	{
		LOG_ERROR("Found sensor %d but could not find motor, please connect and restart",sensor_port);
		gterm_print("Found sensor %d but #ff0000 could not find motor#",sensor_port);
		gterm_print("#ff8000 Please connect motor and restart program#");
		return;
	}
	else
	{
		LOG_ERROR("Could not find motor or sensor, please connect and restart");
		gterm_print("#ff0000 Could not find motor or sensor#");
		gterm_print("#ff8000 Please connect both and restart program#");
		return;
	}

	/* Now run self-test programs whenever a motor is connected */
	while(1)
	{
		/* Wait for motor to be connected */
		bool motor_status;
		do
		{
			motor_status = (registry_get_plugged_type(motor_port-1) == E_DEVICE_MOTOR);
			LOG_INFO("Waiting for motor to be plugged in");
			delay(1000);
		} while (!motor_status);

		delay(1000);
		LOG_INFO("Running self-test sequence now");
		gterm_print("Running self-test sequence");



		for(int i = 0; i < 2; i++)
		{
			/* Configure the motor for green, coast */
			motor_set_brake_mode(motor_port,E_MOTOR_BRAKE_COAST);
			motor_set_gearing(motor_port,E_MOTOR_GEAR_GREEN);
			motor_set_zero_position(motor_port,0.0);
			motor_set_encoder_units(motor_port,E_MOTOR_ENCODER_ROTATIONS);
			motor_set_reversed(motor_port,false);

			/* Reset the sensor */
			rotation_reset_position(sensor_port);

			/* i=0 is forward, i=1 is reverse */
			if(!i)
			{
				/* TEST 1 - Run motor forward for 3 seconds, track position counters and disconnect */
				LOG_INFO("TEST 1 - FORWARD performance");
				gterm_print("TEST 1 - #0000ff FORWARD# performance");
				motor_move_voltage(motor_port,12000);
			}
			else
			{
				/* TEST 2 - Run motor backward for 3 seconds, track position counters and disconnect */
				LOG_INFO("TEST 2 - REVERSE performance");
				gterm_print("TEST 2 - #0000ff REVERSE# performance");
				motor_move_voltage(motor_port,-12000);				
			}
			int cnt_poserr = 0;
			double position_track = 0.0;
			double position_sense = 0.0;
			double speed_track = 0.0;
			double speed_sense = 0.0;
			for(int i = 0; i < (3000 / 10); i++)
			{
				/* Read data from motor and sensor */
				speed_track = motor_get_actual_velocity(motor_port);
				speed_sense = (rotation_get_velocity(sensor_port)*1.0)*(1.0/360.0)*(60.0);
				position_track = motor_get_position(motor_port);
				position_sense = (rotation_get_position(sensor_port)*1.0)*(1.0/36000.0);

				/* If sensed position is different from tracked position, report it */
				if(fabs(position_track - position_sense) > 0.2)
				{
					cnt_poserr++;
				}

				LOG_DEBUG("Measured speed %f, sensed %f",speed_track,speed_sense);
				LOG_DEBUG("Measured position %f, sensed %f",position_track,position_sense);

				delay(10);
			}

			/* Stop the motor after the test */
			motor_move(motor_port,0);

			/* Strings for pass/fail */
			static const char * str_pf[] = {"FAIL","PASS"};
			static const char * str_pf_col[] = {"#ff0000 FAIL#","#00ff00 PASS#"};


			LOG_ALWAYS("REPORT:");
			gterm_print("REPORT:");
			/* Can reach 200rpm (green cartridge)? */
			bool speed_reached = (speed_sense > 200.0);
			if(i) speed_reached = (speed_sense < -200.0);
			LOG_ALWAYS("Motor reached speed of %f > 200? %s",speed_sense,str_pf[speed_reached]);
			gterm_print("Motor reached speed of %f > 200? %s",speed_sense,str_pf_col[speed_reached]);

			/* No position errors during test */
			bool pos_pass = (cnt_poserr == 0);
			LOG_ALWAYS("Motor tracked position matched? %s",str_pf[pos_pass]);
			gterm_print("Motor tracked position matched? %s",str_pf_col[pos_pass]);

		}

		/* Delay 10 seconds before attempting another test */
		delay(10000);
	}


}
