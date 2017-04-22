/*
  BioloidController.h - ArbotiX Library for Bioloid Pose Engine
  Copyright (c) 2008-2012 Michael E. Ferguson.  All right reserved.

  Copyright (c) 2012, 2013 Matthew Paulishen. Copypaster: NaN a.k.a. tician

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef CM9_BC_h
#define CM9_BC_h

#include <Arduino-compatibles.h>
#include "stdlib.h"
#include <Dynamixel.h>
#include <libpandora_types.h>

#ifndef __FLASH__
#define __FLASH__ __attr_flash
#endif

/// Desire some extra resolution, use 16 bits, rather than 10, during interpolation
#define BIOLOID_SHIFT				6

/// BioloidController Pose
#define bc_pose_t unsigned int

/// BioloidController Sequence
typedef struct
{
	bc_pose_t * pose;					// address of pose
	unsigned int time;					// time interval for transition
} bc_seq_t; 							// BC sequence

/// RoboPlus Motion Page compatibility structure
typedef struct
{
	bc_seq_t * steps;					// pointer to sequence of "steps"/poses
	unsigned int next;					// index of next sequence
	unsigned int stop;					// index of stop sequence
} rpm_page_t;							// RPM sequence

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Bioloid Controller Class for CM9 clients.
class BioloidController
{
public:
/// Constructor and setup
	/**
	 * Constructor for BioloidController Library.
	 * @see setup()
	 */
	BioloidController() {};
	/**
	 * Basic setup function for BioloidController object.  Allocates memory 
	 * and initializes variables.
	 * @param servo_count The number of servos to be controlled by object.
	 */
	void setup(unsigned int servo_count);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Debug/Emergency controls
	/**
	 * Debug function to suspend the motion engine.
	 * @see pause()
	 */
	void suspend() { pause(true); }	// set bcState_ = PAUSED
	/**
	 * Debug function to resume the motion engine.
	 * @see pause()
	 */
	void resume() { pause(false); }	// set bcState_ = RUNNING

	/**
	 * Debug function to suspend, resume, or restart the motion engine.
	 * @see suspend()
	 * @see resume()
	 */
	bool pause(bool);
	/**
	 * Emergency Stop function.  Stop the motion engine and sends broadcast
	 * message to disable torque on all servos.  Use resume() to restart engine.
	 * @see resume()
	 * @see pause()
	 */
	void kill(void);					// set bcState_ = KILLED


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Pose Manipulation functions
	/**
	 * Load a named pose from FLASH memory.
	 * @see writePose()
	 * @param addr Pointer to a pose (an array of bc_pose_t).
	 */
	void loadPose( bc_pose_t * addr );
	/**
	 * Read present position from each servos and store it in RAM as current pose.
	 */
	void readPose();
	/**
	 * Write current pose stored in RAM out to servos.
	 * @see loadPose()
	 */
	void writePose();

	/**
	 * Get servo position value from pose currently stored in RAM.
	 * @param id The numerical ID of the target servo.
	 * @return The position value of the target servo in the pose.
	 */
	int getCurPose(int id);	
	/**
	 * Get servo position value from "next" pose currently stored in RAM.
	 * @param id The numerical ID of the target servo.
	 * @return The position value of the target servo in the "next" pose.
	 */
	int getNextPose(int id);
	/**
	 * Set the servo position value in the "next" pose currently stored in RAM.
	 * @param id The numerical ID of the target servo.
	 * @param pos The position value of the target servo.
	 */
	void setNextPose(int id, int pos);

	/**
	 * Set the number of servos used in the pose currently stored in RAM.
	 * @param num The number of servos to be used.
	 */
	void setPoseSize(unsigned int num);
	/**
	 * Get the number of servos used in the pose currently stored in RAM.
	 * @return The number of servos used by the pose.
	 */
	unsigned int getPoseSize();


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Servo Manipulation functions
	/**
	 * Set the numerical ID of a particular index in the ID storage array.
	 * @param index The index of the servo ID storage array to be changed.
	 * @param id The numerical ID to be assigned at that index.
	 */
	void setId(int index, int id);	
	/**
	 * Get the numerical ID at a particular index in the ID storage array.
	 * @param The index of the ID storage array to be returned.
	 * @return The numerical ID of the servo at that index.
	 */
	int getId(int index);
	/**
	 * Set the 'resolution' of servos used in the object.  Essentially the maximum position value (1023 for AX/RX/EX servos and 4096 for MX servos).
	 * @param id The numerical ID of the servo to have its 'resolution' modified.
	 * @param res The desired 'resolution' of the servo.
	 * @return The actual 'resolution' of the servo used by the engine.
	 */
	unsigned int setResolution(unsigned int id, unsigned int res);

	/**
	 * Load joint calibration offsets to enable reuse of poses by multiple robots with servo positional control variations.
	 * @param addr The name/pointer of the int array containing the offset values.
	 */
	void loadOffsets( int * addr );


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Pose Interpolation
	/**
	 * Function to initiate an interpolating transition from current servo positions to the goal positions in the previously loaded pose.  Calls readPose() to ensure that start positions of servos in engine match actual positions of servos to prevent erroneous jumps to a "starting" pose.
	 * @param time The time taken to move from current pose to goal pose.
	 * @see loadPose()
	 * @see readPose()
	 */
	void interpolateSetup(unsigned int time);
	/**
	 * Moves forward one time step in the current interpolation.  When called, it will wait for time interval to complete before writing the next intermediate pose to the servos.  If interpolation is finished, will return immediately.  The time interval is modifiable with the setFrameLength() and setTimeModifier() functions.
	 * @see setFrameLength()
	 * @see setTimeModifier()
	 */
	void interpolateStep();
	/**
	 * Function to check if the engine is currently transitioning to a goal pose.
	 * @return Boolean state of interpolation.
	 */
	bool interpolating(void);
	/**
	 * Function to set the engine interpolation state.  Can be used to stop a transition to a goal pose, but cannot restart the transition.
	 * @param bolly Desired state of interpolation.
	 * @return Boolean state of interpolation.
	 */
	bool interpolating(bool bolly);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Sequence Engine
	/**
	 * Load a sequence from FLASH and begin playing it.
	 * @param addr Pointer to a sequence (an array of bc_seq_t).
	 * @see play()
	 */
	void playSeq( bc_seq_t * addr );
	/**
	 * Move forward in sequence to next intermediate pose and/or next pose in the sequence.  Checks whether goal pose has been reached, an calls loadPose(), interpolateSetup(), interpolateStep(), or returns immediately depending on whether it is starting/continuing a sequence or finishing a sequence.
	 * @see loadPose()
	 * @see interpolateSetup()
	 * @see interpolateStep()
	 */
	void play();
	/**
	 * Function to check if the engine is currently playing a sequence.
	 * @return Boolean state of sequence.
	 */
	bool playing(void);
	/**
	 * Function to set the engine sequence playing state.  Can be used to stop a sequence, but cannot restart the sequence.
	 * @param bolly Desired state of sequence playing.
	 * @return Boolean state of sequence playing.
	 */
	bool playing(bool bolly);
	/**
	 * Function to return pointer of currently running sequence.
	 * @return Pointer to current sequence.
	 */
	bc_seq_t* checkSeq() {return sequence_;}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// RoboPlus Compatibility functions
	/**
	 * Starts of series of motion "pages" from the RoboPlusMotion_Array.
	 * @param index Index of "page" to play (may be identical to that used in Task).
	 */
	void MotionPage(unsigned int page_index);
	/**
	 * Wrapper function for MotionPage(unsigned int).
	 * @param index Index of "page" to play (may be identical to that used in Task).
	 * @see MotionPage(unsigned int)
	 */
	void setMotionPage(unsigned int page_index) { MotionPage(page_index); }
	/**
	 * Check the status of the currently running motion "page".
	 * @return Boolean state of motion engine.
	 */
	bool MotionStatus(void);
	/**
	 * Retrieve the index of the currently running motion "page".
	 * @return The index of the currently running motion "page".
	 */
	unsigned int MotionPage();
	/**
	 * Wrapper function for MotionPage().
	 * @return The index of the currently running motion "page".
	 * @see MotionPage()
	 */
	unsigned int getMotionPage() { return MotionPage(); }
	/**
	 * Continue playing a motion "page".  Must be called periodically to progress to next intermediate pose, next goal pose, and/or next sequence.
	 * @see MotionPage(unsigned int)
	 * @see playSeq()
	 * @see play()
	 * @see loadPose()
	 * @see interpolateSetup()
	 * @see interpolateStep()
	 */
	void Play();
	/**
	 * Load a RoboPlusMotion_Array into the object, allocate memory, initialize variables, and set servo IDs to those stored in FLASH.
	 * @param Pointer to RoboPlusMotion_Array.
	 */
	void RPM_Setup(rpm_page_t* array);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Engine Modifiers
	/**
	 * Set the integer time modifier.  100(%) is default.
	 * @param mult The desired time modifier.
	 * @return The actual time modifier.
	 */
	unsigned int setTimeModifier(unsigned int mult);
	/**
	 * Set the motion engine's interpolation time length (time between each intermediate pose calculation and write to servos).
	 * @param mult The desired time length.
	 * @return The actual time length.
	 */
	unsigned int setFrameLength(unsigned int time);



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
private:
	// Servo IDs for this BioloidController object
	unsigned char * id_;
	// Present servo positions
	bc_pose_t * pose_;
	// Final desired servo positions
	bc_pose_t * nextpose_;
	// Change in each servo position per interpolation step
	int * deltas_;
	// Calibration offsets for each servo
	int * offsets_;
	// 'Resolution' of each servo
	unsigned int * resolutions_;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Number of servos used by current pose.
	unsigned int poseSize_;
	// Number of servos controllable by this BioloidController object
	unsigned int numServos_;

	// Time {from millis()} when last position change occurred
	unsigned long lastframe_;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Currently running sequence
	bc_seq_t * sequence_;
	// Number of transitions remaining in current interpolation
	unsigned int transitions_;
	unsigned int seqIndex_;

	// MotionEngine control state
	enum
	{
		INTERPOLATING,
		INTERPOLATION_DONE,
		SEQUENCE_DONE
	} seqState_;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// RoboPlusMotion Array pointer
	rpm_page_t * rpmArray_;
	// RoboPlusMotion Array index
	unsigned int rpmIndexNow_;
	unsigned int rpmIndexInput_;

	// RoboPlusMotion control state
	enum
	{
		PLAYING,
		STOPPING,
		STOPPED
	} rpmState_;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Debug/Emergency control state
	enum
	{
		KILLED,
		PAUSED,
		RUNNING
	} bcState_;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Temporal modifier
	unsigned int timeModder_;
	// Length (in milliseconds) of each step in interpolation
	unsigned int frameLength_;
	
};
#endif
