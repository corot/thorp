/*
  BioloidController.cpp - ArbotiX Library for Bioloid Pose Engine
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

#include "CM9_BC.h"

extern Dynamixel *pDxl;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Newer BioloidController setup function
void BioloidController::setup(unsigned int servo_count)
{
	if (servo_count < 1)
		numServos_ = 20;
	else
		numServos_ = servo_count;

	// Allocate storage
	id_ = (unsigned char *) malloc(numServos_ * sizeof(unsigned char));
	pose_ = (unsigned int *) malloc(numServos_ * sizeof(unsigned int));
	nextpose_ = (unsigned int *) malloc(numServos_ * sizeof(unsigned int));
	deltas_ = (int *) malloc(numServos_ * sizeof(int));
	offsets_ = (int *) malloc(numServos_ * sizeof(int));
	resolutions_ = (unsigned int *) malloc(numServos_ * sizeof(unsigned int));

	// Initialize everything
	unsigned int iter;
	for(iter=0;iter<numServos_;iter++)
	{
		// Assumes AX/RX/EX series servos
		id_[iter] = iter+1;
		pose_[iter] = 512 << BIOLOID_SHIFT;
		nextpose_[iter] = 512 << BIOLOID_SHIFT;
		offsets_[iter] = 0;
		resolutions_[iter] = 1023;
	}
	
	poseSize_ = 0;
	lastframe_ = millis();
	frameLength_ = 8;//32;	// default is 33[ms] to get ~30[Hz] update rate
	timeModder_ = 100;
	transitions_ = 0;

	bcState_ = RUNNING;
	seqState_ = SEQUENCE_DONE;
	rpmState_ = STOPPED;

	rpmIndexNow_ = 0;
	rpmIndexInput_ = 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Load a named pose from FLASH into nextpose.
void BioloidController::loadPose( bc_pose_t * addr )
{
	unsigned int servo_count = addr[0];

	if (servo_count > numServos_)
	{
		SerialUSB.print("The pose you attempted to load requires more servos than the object allocated during setup. Doing this leads to bad, bad things...\n");
		return;
	}
	poseSize_ = servo_count;

	unsigned int iter;
	for(iter=0; iter<poseSize_; iter++)
	{
		nextpose_[iter] = addr[1+iter] << BIOLOID_SHIFT;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Read current servo positions and store to the pose.
void BioloidController::readPose()
{
	unsigned int iter;
	for(iter=0; iter<numServos_; iter++)
	{
		unsigned int temp = pDxl->readWord(id_[iter], P_PRESENT_POSITION_L);
		if (pDxl->getResult()==(1<<COMM_RXSUCCESS))
		{
			if ((temp < resolutions_[iter]) && (temp > 0))
				pose_[iter] = temp << BIOLOID_SHIFT;
			else
				pose_[iter] = (resolutions_[iter]>>1) << BIOLOID_SHIFT;
		}
		else
		{
			SerialUSB.print("*** Servo ID=");SerialUSB.print(id_[iter]);SerialUSB.print(" not found ***\n");
		}
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Write the next pose out to servos using sync write.
void BioloidController::writePose()
{
	// Were we paused or killed?
	if (bcState_ != RUNNING)
		return;

	unsigned int temp;
	unsigned int numParams = 2 + (poseSize_ * 3);   // 3 = id + GOAL_L + GOAL_H

	pDxl->setTxPacketId( BROADCAST_ID );
	pDxl->setTxPacketInstruction( INST_SYNC_WRITE );
	pDxl->setTxPacketLength( numParams );
	pDxl->setTxPacketParameter( 0, P_GOAL_POSITION_L );
	pDxl->setTxPacketParameter( 1, 2 );	// writing two bytes

	unsigned int iter;
	for(iter=0; iter<poseSize_; iter++)
	{
		temp = (pose_[iter] >> BIOLOID_SHIFT) + offsets_[iter];
		if (temp < 0)
			temp = 0;
		else if (temp > resolutions_[iter])
			temp = resolutions_[iter];
		pDxl->setTxPacketParameter( 2 + 3*iter + 0, id_[iter] );
		pDxl->setTxPacketParameter( 2 + 3*iter + 1, (temp)&0xff );
		pDxl->setTxPacketParameter( 2 + 3*iter + 2, (temp>>8)&0xff );
	}
	pDxl->txrxPacket();
}







//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Manually set a servo ID
void BioloidController::setId(int index, int id)
{
	id_[index] = id;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Check a servo ID
int BioloidController::getId(int index)
{
	return id_[index];
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Load servo offsets from FLASH
void BioloidController::loadOffsets(int * addr)
{
	int num_servos;
	if (addr[0] < numServos_)
		num_servos = addr[0];
	else
		num_servos = numServos_;

	int iter;
	for (iter=0; iter < num_servos; iter++)
	{
		offsets_[iter] = addr[iter+1];
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Set resolution of a servo
unsigned int BioloidController::setResolution(unsigned int id, unsigned int res)
{
	if (res > 4096)
		return 0;

	unsigned int iter;
	for(iter=0; iter<numServos_; iter++)
	{
		if( id_[iter] == id )
		{
			resolutions_[iter] = res;
			return res;
		}
	}
	return 0;
}






//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Set number of servos in current pose
void BioloidController::setPoseSize(unsigned int num)
{
	if (num > numServos_)
	{
		SerialUSB.print("You attempted to set the pose size to have more servos than the object allocated during setup. Doing this leads to bad, bad things...\n");
		return;
	}
	poseSize_ = num;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Get number of servos in current pose
unsigned int BioloidController::getPoseSize()
{
	return poseSize_;
}





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Get a servo value from the current pose
int BioloidController::getCurPose(int id)
{
	unsigned int iter;
	for(iter=0; iter<numServos_; iter++)
	{
		if( id_[iter] == id )
			return ((pose_[iter]) >> BIOLOID_SHIFT);
	}
	return -1;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Get a servo value from the goal pose
int BioloidController::getNextPose(int id)
{
	unsigned int iter;
	for(iter=0; iter<numServos_; iter++)
	{
		if( id_[iter] == id )
			return ((nextpose_[iter]) >> BIOLOID_SHIFT);
	}
	return -1;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Set a servo value in the goal pose
void BioloidController::setNextPose(int id, int pos)
{
	unsigned int iter;
	for(iter=0; iter<numServos_; iter++)
	{
		if( id_[iter] == id )
		{
			nextpose_[iter] = (pos << BIOLOID_SHIFT);
			return;
		}
	}
}







//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Setup for interpolating from pose to nextpose over TIME milliseconds
void BioloidController::interpolateSetup(unsigned int time)
{
	time = (int) (time * timeModder_)/100;
	int frames = (time/frameLength_) + 1;
	lastframe_ = millis();

	// Make sure that pose_ contains accurate position information of servos
		// If we do not check here and user did not call in sketch, then can
		// have large and unexpected jump before starting smooth interpolation
	readPose();

	// Set deltas for each servo...
	unsigned int iter;
	for(iter=0; iter<poseSize_; iter++)
	{
		//deltas_[iter] = (nextpose_[iter] - pose_[iter])/frames) + 1;
		// (nextpose_[iter]-pose_[iter]) is unsigned and can cause issues if attempt in single operation
		deltas_[iter] = (nextpose_[iter] - pose_[iter]);
//		deltas_[iter] = (deltas_[iter]/abs(frames-1));
//		deltas_[iter] = (deltas_[iter]/frames);
		if (deltas_[iter]>0)
			deltas_[iter] = (deltas_[iter]/frames) + 1;
		else if (deltas_[iter]<0)
			deltas_[iter] = (deltas_[iter]/frames) - 1;
	}
	// Start transitioning between current pose and goal pose
	seqState_ = INTERPOLATING;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Write next intermediate pose of the interpolation
void BioloidController::interpolateStep()
{
	// Were we paused or killed?
	if (bcState_ != RUNNING)
		return;

	// Are we transitioning between current pose and a goal pose?
	if (seqState_ != INTERPOLATING)
		return;

	int complete = poseSize_;
	while( (millis() - lastframe_) < frameLength_ );
	lastframe_ = millis();

	// Update intermediate goal position of each servo
	unsigned int iter;
//	SerialUSB.print("[servo,diff,delta]\n  ");
	for(iter=0; iter<poseSize_; iter++)
	{
		int diff = nextpose_[iter] - pose_[iter];
		if(diff == 0)
		{
			complete--;
		}
		else
		{
			if (abs(diff) > abs(deltas_[iter]))
			{
				if ( 	( (diff>0) && (deltas_[iter]>0) ) ||
						( (diff<0) && (deltas_[iter]<0) ) )
				{
					pose_[iter] += deltas_[iter];
				}
				else
				{
					pose_[iter] -= (deltas_[iter]/2);
				}
//				SerialUSB.print("[");
//				SerialUSB.print(id_[iter]);SerialUSB.print(",");
//				SerialUSB.print(diff);SerialUSB.print(",");
//				SerialUSB.print(deltas_[iter]);SerialUSB.print("] ");
			}
			else
			{
				pose_[iter] = nextpose_[iter];
				complete--;
			}
		}
	}
//	SerialUSB.print("\n");
//	SerialUSB.print("Complete: ");SerialUSB.println(complete);
	// Did we finish moving all servos to their values in the goal pose.
	if(complete <= 0)
	{
		// Wait for new pose to be loaded.
		seqState_ = INTERPOLATION_DONE;
	}

	writePose();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Are we interpolating?
bool BioloidController::interpolating(void)
{
	if (seqState_ == INTERPOLATING)
		return true;
	else
		return false;
}
bool BioloidController::interpolating(bool bolly)
{
	if (!bolly)
		seqState_ = INTERPOLATION_DONE;
	return interpolating();
}







//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Load and begin playing a sequence.
void BioloidController::playSeq( bc_seq_t * addr )
{
	sequence_ = addr;

	seqIndex_ = 0;
	// Number of poses in sequence
	transitions_ = sequence_[seqIndex_].time;
	seqIndex_++;

	loadPose( sequence_[seqIndex_].pose );
//	int time = (int) (sequence_[seqIndex_].time * timeModder_)/100;
//	interpolateSetup( time );
	interpolateSetup( sequence_[seqIndex_].time );

	// Start transitioning between current pose and goal pose
	seqState_ = INTERPOLATING;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Keep playing the current sequence.
void BioloidController::play()
{
	// Were we paused or killed?
	if (bcState_ != RUNNING)
		return;

	// No poses or sequences loaded and ready to play.
	if (seqState_ == SEQUENCE_DONE)
		return;

	// Still transitioning to goal pose.
	if (seqState_ == INTERPOLATING)
	{
		interpolateStep();
	}
	// Fished transition to goal pose, need to load next goal pose.
	else if (seqState_ == INTERPOLATION_DONE)
	{
		seqIndex_++;
		// Still more poses in the current sequence.
		if (seqIndex_ <= transitions_)
		{
			loadPose( sequence_[seqIndex_].pose );
			
//			int time = (int) (sequence_[seqIndex_].time * timeModder_)/100;
//			interpolateSetup( time );
			interpolateSetup( sequence_[seqIndex_].time );
		}
		// Current sequence is finished.
		else
		{
			// Start waiting for new pose/sequence to be loaded.
			seqState_ = SEQUENCE_DONE;
		}
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Are we playing a sequence?
bool BioloidController::playing(void)
{
	if (seqState_ != SEQUENCE_DONE)
		return true;
	else
		return false;
}
bool BioloidController::playing(bool bolly)
{
	if (!bolly)
		seqState_ = SEQUENCE_DONE;
	return playing();
}




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Load a RoboPlusMotion_Array
void BioloidController::RPM_Setup(rpm_page_t* array)
{
	rpmArray_ = array;

	// Load servo IDs from sequence #1 of RoboPlusMotion file
	bc_seq_t *ref_seq = rpmArray_[1].steps;
	unsigned int *servo_ids = ref_seq[0].pose;
	unsigned int num_servos = servo_ids[0];
	setup(num_servos);

	unsigned int iter;
//	SerialUSB.println("Setting Servo IDs");
	for (iter=0; iter<numServos_; iter++)
	{
		id_[iter] = servo_ids[iter+1];
//		SerialUSB.print(" Servo #");SerialUSB.print(iter);
//		SerialUSB.print(" is ID=");SerialUSB.println(id_[iter]);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Check status of motions
bool BioloidController::MotionStatus(void)
{
	if (rpmState_ == STOPPED)
		return false;
	else
		return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Check currently running motion page from RoboPlusMotion_Array
unsigned int BioloidController::MotionPage()
{
	return rpmIndexNow_;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Start a (series of) motion sequences from RoboPlusMotion_Array
void BioloidController::MotionPage(unsigned int page_index)
{
	if (page_index > rpmArray_[0].stop)
		return;

	if (rpmArray_[page_index].steps == 0)
	{
		SerialUSB.print("RPM Page ");SerialUSB.print(page_index);SerialUSB.print(" does not exist.\n");
	
		return;
	}
	rpmIndexInput_ = page_index;

	// Is the current sequence valid?
	if (rpmIndexNow_ == 0)
	{
		rpmState_ = STOPPED;
	}

	// Are we still running a sequence?
	if (rpmState_ != STOPPED)
	{
		// Check if input is stop.
		if (rpmIndexInput_ == 0)
		{
			// If input==0, finish current sequence, then
			//  run the stop sequence specified by current sequence
			rpmState_ = STOPPING;
		}
/*
		else if (rpmIndexNow_ != rpmIndexInput_)
		{
			// Probably want to start new RPM series?
			rpmState_ = STOPPING;
		}
*/

//		else if (rpmIndexNow_ != rpmIndexInput_)
		else
		{
			// Does the currently running sequence follow the input sequence
			//  in the RPM series?
			unsigned int next = rpmArray_[rpmIndexInput_].next;
			if (	(rpmIndexNow_ == next) ||					// 1st gen child of input
					(rpmIndexNow_ == rpmArray_[next].next)	)	// 2nd gen child of input
			{
				// Does current sequence loop into itself?
				if (rpmIndexNow_ == rpmArray_[rpmIndexNow_].next)
				{
					// Probably want to restart RPM series?
					rpmState_ = STOPPING;
				}
				// Otherwise, continue running current RPM series of sequences
				//  instead of stopping then restarting same RPM series.
			}
			else
			{
				// Likely not in the same RPM series, so finish out this RPM
				//  series (progress through stop indices) and then start the
				//  new RPM series the user input.
				rpmState_ = STOPPING;
			}
		}

	}
	else
	{
		// Check if input is not stop.
		if (rpmIndexInput_ > 0)
		{
			// Start user's input sequence
			rpmIndexNow_ = rpmIndexInput_;
			rpmIndexInput_ = 0;
			rpmState_ = PLAYING;
			playSeq(rpmArray_[rpmIndexNow_].steps);
		}
		// Do nothing otherwise (told to stop when already stopped)
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Keep playing a RoboPlusMotion series of sequences
void BioloidController::Play()
{
	if (bcState_ != RUNNING)
		return;

	// Has the current RPM series of sequences been finished?
	if (rpmState_ == STOPPED)
	{
		// Is there a user sequence waiting to be started?
		if (rpmIndexInput_ > 0)
		{
			rpmIndexNow_ = rpmIndexInput_;
			rpmIndexInput_ = 0;
			rpmState_ = PLAYING;
			playSeq(rpmArray_[rpmIndexNow_].steps);
		}
		return;
	}

	// Has current sequence in RPM series finished and ready for next sequence
	//  in RPM series to be loaded?
	if (seqState_ == SEQUENCE_DONE)
	{
		// Is there more to process in this RPM series of sequences?
		if (rpmIndexNow_ > 0)
		{
			// Need to run stop sequence indicated by current sequence
			if (rpmState_ == STOPPING)
			{
				rpmIndexNow_ = rpmArray_[rpmIndexNow_].stop;
			}
			// Need to run next sequence indicated by current sequence
			else if (rpmState_ == PLAYING)
			{
				rpmIndexNow_ = rpmArray_[rpmIndexNow_].next;
			}
		}

		// No stop/next sequence specified by current sequence (done moving)
		if (rpmIndexNow_ == 0)
		{
			// Is there a user sequence waiting to be started?
			if (rpmIndexInput_ > 0)
			{
				rpmIndexNow_ = rpmIndexInput_;
				rpmIndexInput_ = 0;
				rpmState_ = PLAYING;
			}
			// Nothing left to do in this RPM series and nothing to be started.
			else
			{
				rpmState_ = STOPPED;
				return;
			}
		}

		// Load next sequence in series (next or stop) or user input
		playSeq(rpmArray_[rpmIndexNow_].steps);
	}
	else
	{
		// Otherwise, continue the next transition in the current sequence.
		play();
	}
}






//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Pause the motion engine
bool BioloidController::pause(bool bolly)
{
	if (bolly)
	{
		bcState_ = PAUSED;
	}
	else
	{
		bcState_ = RUNNING;
	}
	return bcState_;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Emergency Stop
void BioloidController::kill(void)
{
	bcState_ = KILLED;
	pDxl->writeByte(BROADCAST_ID, P_TORQUE_ENABLE, 0);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Set the Temporal modifier
/*
float BioloidController::setTimeModifier(float mult)
{
	if ((mult > 0.01) && (mult < 2.0))
		timeModder_ = mult;
	return timeModder_;
}
*/
unsigned int BioloidController::setTimeModifier(unsigned int mult)
{
	if ((mult > 50) && (mult < 1000))
		timeModder_ = mult;
	return timeModder_;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Set the interpolation time length
unsigned int BioloidController::setFrameLength(unsigned int time)
{
	if (time > 5)
		frameLength_ = time;
	return frameLength_;
}


