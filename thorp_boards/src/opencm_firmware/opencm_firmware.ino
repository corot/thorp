/* 
 CM-9 Test Program for use with PyPose
 Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

 Copyright (c) 2013 Matthew Paulishen. Copypaster: NaN a.k.a. tician

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

#include <CM9_BC.h>
Dynamixel Dxl(1);
Dynamixel *pDxl = &Dxl;
#define MAX_NUM_SERVOS          5
BioloidController bioloid;

#define ARB_SIZE_POSE           7               // also initializes
#define ARB_LOAD_POSE           8
#define ARB_LOAD_SEQ            9
#define ARB_PLAY_SEQ            10
#define ARB_LOOP_SEQ            11
#define ARB_TEST                25

int mode = 0; // where we are in the frame

unsigned char id = 0; // id of this frame
unsigned char length = 0; // length of this frame
unsigned char ins = 0; // instruction of this frame

unsigned char params[143]; // parameters (match RX-64 buffer size)
unsigned char index = 0; // index in param buffer

int checksum; // checksum

typedef struct
{
  bc_pose_t pose; // index of pose to transition to
  int time; // time for transition
} sp_trans_t;

// pose and sequence storage
bc_pose_t poses[30][MAX_NUM_SERVOS]; // poses [index][servo_id-1]
sp_trans_t sequence[50]; // sequence
int seqPos; // step in current sequence     

void blink(unsigned int times = 1)
{
  for (int i = 0; i < times * 2; i++)
  {
    toggleLED();
    delay(100);
  }
}

void setup()
{
  Dxl.begin(1);
  SerialUSB.begin();

  bioloid.setup(MAX_NUM_SERVOS);
  pinMode(BOARD_LED_PIN, OUTPUT);
}

/*
 * packet: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix
 *
 * ID = 253 for these special commands!
 * Pose Size = 7, followed by single param: size of pose
 * Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
 * Seq Size = 9, followed by single param: size of seq
 * Load Seq = A, followed by index/times (# of parameters = 3*seq_size) 
 * Play Seq = B, no params
 * Loop Seq = C, 
 */

void loop()
{
  // process messages
  while (SerialUSB.available() > 0)
  {
    // We need to 0xFF at start of packet
    if (mode == 0) // start of new packet
    {
      if (SerialUSB.read() == 0xff)
      {
        // mode = 2;
        mode = 1;
        // digitalWrite(0,HIGH-digitalRead(0));
      }
    }
    else if (mode == 1) // another start byte
    {
      if (SerialUSB.read() == 0xff)
        mode = 2;
      else
        mode = 0;
    }
    else if (mode == 2) // next byte is index of servo
    {
      id = SerialUSB.read();
      if (id != 0xff)
        mode = 3;
    }
    else if (mode == 3) // next byte is length
    {
      length = SerialUSB.read();
      checksum = id + length;
      mode = 4;
    }
    else if (mode == 4) // next byte is instruction
    {
      ins = SerialUSB.read();
      checksum += ins;
      index = 0;
      mode = 5;
    }
    else if (mode == 5) // read data in
    {
      params[index] = SerialUSB.read();
      checksum += (int)params[index];
      index++;
      if (index + 1 == length) // we've read params & checksum
      {
        mode = 0;
        if ((checksum & 0xFF) != 255)
        {
          // return a packet: FF FF id Len Err params=None check
          SerialUSB.write(0xff);
          SerialUSB.write(0xff);
          SerialUSB.write(id);
          SerialUSB.write(2);
          SerialUSB.write(64);
          SerialUSB.write(255 - ((66 + id) & 0xFF));
        }
        else
        {
          if (id == 253)
          {
            // return a packet: FF FF id Len Err params=None check
            SerialUSB.write(0xff);
            SerialUSB.write(0xff);
            SerialUSB.write(id);
            SerialUSB.write(2);
            SerialUSB.write((unsigned char)0);
            SerialUSB.write(255 - ((2 + id) & 0xFF));
        // SerialUSB.write(ins);
        // SerialUSB.write(255-((2+ins+id)&0xFF));
            // special ArbotiX instructions
            // Pose Size = 7, followed by single param: size of pose
            // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
            // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size)
            // Play Seq = A, no params
            if (ins == ARB_SIZE_POSE)
            {
              bioloid.setPoseSize(params[0]);
              bioloid.readPose();
              //SerialUSB.println(poseSize);
            }
            else if (ins == ARB_LOAD_POSE)
            {
              int poseSize = bioloid.getPoseSize();
              int iter;
//                                                      SerialUSB.print("New Pose:");
              for (iter = 0; iter < poseSize; iter++)
              {
                poses[params[0]][iter] = (params[(2 * iter) + 1] + (params[(2 * iter) + 2] << 8));
//                                                              SerialUSB.print(poses[params[0]][iter]);
//                                                              SerialUSB.print(",");
              }
//                                                      SerialUSB.println("");
            }
            else if (ins == ARB_LOAD_SEQ)
            {
              int iter;
              for (iter = 0; iter < (length - 2) / 3; iter++)
              {
                sequence[iter].pose = params[(iter * 3)];
                sequence[iter].time = params[(iter * 3) + 1] + (params[(iter * 3) + 2] << 8);
                //SerialUSB.print("New Transition:");
                //SerialUSB.print((int)sequence[iter].pose);
                //SerialUSB.print(" in ");
                //SerialUSB.println(sequence[iter].time);
              }
            }
            else if (ins == ARB_PLAY_SEQ)
            {
              seqPos = 0;
              while (sequence[seqPos].pose != 0xff)
              {
                // are we HALT?
                if (SerialUSB.available())
                {
                  if (SerialUSB.read() == 'H')
                    return;
                }

                int p = sequence[seqPos].pose;
                int poseSize = bioloid.getPoseSize();

                // load pose
                int iter;
                for (iter = 0; iter < poseSize; iter++)
                {
                  bioloid.setNextPose(iter + 1, poses[p][iter]);
                }
                // interpolate
                bioloid.interpolateSetup(sequence[seqPos].time);
                while (bioloid.interpolating())
                {
                  bioloid.interpolateStep();
                }
                // next transition
                seqPos++;
              }
            }
            else if (ins == ARB_LOOP_SEQ)
            {
              while (1)
              {
                seqPos = 0;
                while (sequence[seqPos].pose != 0xff)
                {
                  // are we HALT?
                  if (SerialUSB.available())
                  {
                    if (SerialUSB.read() == 'H')
                      return;
                  }

                  int p = sequence[seqPos].pose;
                  int poseSize = bioloid.getPoseSize();

                  // load pose
                  int iter;
                  for (iter = 0; iter < poseSize; iter++)
                  {
                    bioloid.setNextPose(iter + 1, poses[p][iter]);
                  }
                  // interpolate
                  bioloid.interpolateSetup(sequence[seqPos].time);
                  while (bioloid.interpolating())
                    bioloid.interpolateStep();
                  // next transition
                  seqPos++;
                }
              }
            }
            else if (ins == ARB_TEST)
            {
              /*                                                      int i;
               // Test Digital I/O
               for(i=0;i<8;i++)
               {
               // test digital
               pinMode(i,OUTPUT);
               digitalWrite(i,HIGH);
               // test analog
               pinMode(31-i,OUTPUT);
               digitalWrite(31-i,HIGH);
               
               delay(500);
               digitalWrite(i,LOW);
               digitalWrite(31-i,LOW);
               }
               // Test Ax-12
               for(i=452;i<552;i+=20)
               {
               SetPosition(1,i);
               delay(200);
               }
               // Test Motors
               drive.set(-255,-255);
               delay(500);
               drive.set(0,0);
               delay(1500);
               drive.set(255,255);
               delay(500);
               drive.set(0,0);
               delay(1500);
               // Test Analog I/O
               for(i=0;i<8;i++)
               {
               // test digital
               pinMode(i,OUTPUT);
               digitalWrite(i,HIGH);
               // test analog
               pinMode(31-i,OUTPUT);
               digitalWrite(31-i,HIGH);
               
               delay(500);
               digitalWrite(i,LOW);
               digitalWrite(31-i,LOW);
               }
               */}
          }
          else
          {
            blink(1);
            // pass thru
            if (ins == INST_READ)
            {
              Dxl.setTxPacketId(id);
              Dxl.setTxPacketInstruction(INST_READ);
              Dxl.setTxPacketParameter(0, params[0]);
              Dxl.setTxPacketParameter(1, params[1]);
              Dxl.setTxPacketLength(2);
              Dxl.txrxPacket();
              // return a packet: FF FF id Len Err params check
              if (Dxl.getResult() == (1 << COMM_RXSUCCESS))
              {
                int lennie = Dxl.getRxPacketLength() + 4;
                if (lennie >= 6)
                {
                  int iter;
                  for (iter = 0; iter < lennie; iter++)
                  {
                    SerialUSB.write(Dxl.getRxPacketParameter(iter - 5));
                  }
                }
              }
              else
              {
                SerialUSB.write(0xff);
                SerialUSB.write(0xff);
                SerialUSB.write(id);
                SerialUSB.write(2);
                SerialUSB.write(128);
                SerialUSB.write(255 - ((130 + id) & 0xFF));
              }
            }
            else if (ins == INST_WRITE)
            {
              if (length == 4)
              {
                Dxl.writeByte(id, params[0], params[1]);
              }
              else
              {
                int x = Dxl.makeWord(params[1], params[2]);
                Dxl.writeWord(id, params[0], x);
              }
              // return a packet: FF FF id Len Err params check
              if (Dxl.getResult() == (1 << COMM_RXSUCCESS))
              {
                int lennie = Dxl.getRxPacketLength() + 4;
                if (lennie >= 6)
                {
                  int iter;
                  for (iter = 0; iter < lennie; iter++)
                  {
                    SerialUSB.write(Dxl.getRxPacketParameter(iter - 5));
                  }
                }
              }
              else
              {
                SerialUSB.write(0xff);
                SerialUSB.write(0xff);
                SerialUSB.write(id);
                SerialUSB.write(2);
                SerialUSB.write(128);
                SerialUSB.write(255 - ((130 + id) & 0xFF));
              }

            }
          }
        }
      }
    }
  }

  // update joints
  bioloid.interpolateStep();
}
