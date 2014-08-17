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

#include "opencm_firmware.h"

Dynamixel Dxl(1);
Dynamixel *pDxl = &Dxl;
BioloidController bioloid;

#define MAX_NUM_SERVOS  5

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

/*
 * Handle a read from ArbotiX registers.
 */
int handleRead()
{
  int checksum = 0;
  int addr = params[0];
  int bytes = params[1];

  unsigned char v;
  while (bytes > 0)
  {
    if (addr == REG_MODEL_NUMBER_L)
    {
      v = 44;
    }
    else if (addr == REG_MODEL_NUMBER_H)
    {
      v = 1; // 300 
    }
    else if (addr == REG_VERSION)
    {
      v = 0;
    }
    else if (addr == REG_ID)
    {
      v = 253;
    }
    else if (addr == REG_BAUD_RATE)
    {
      v = 34; // 56700
    }
    else if (addr == REG_DIGITAL_IN0)
    {
      // digital 0-7
      v = digitalRead(addr - REG_DIGITAL_IN0)
    }
    else if (addr == REG_DIGITAL_IN1)
    {
      // digital 8-15
      v = digitalRead(addr - REG_DIGITAL_IN0)
    }
    else if (addr == REG_DIGITAL_IN2)
    {
      // digital 16-23
      v = digitalRead(addr - REG_DIGITAL_IN0)
    }
    else if (addr == REG_DIGITAL_IN3)
    {
      // digital 24-31
      v = digitalRead(addr - REG_DIGITAL_IN0)
    }
    else if (addr == REG_RETURN_LEVEL)
    {
//      v = ret_level;
    }
    else if (addr == REG_ALARM_LED)
    {
      // TODO
    }
    else if (addr < REG_SERVO_BASE)
    {
      // send analog reading as 1 or 2 bytes
      int x = analogRead(addr - REG_ANA_BASE);
      if (bytes == 2)
      {
        v = x;
        checksum += v;
        SerialUSB.write(v);
        bytes--;
        v = x >> 8;
      }
      else
        v = x / 20;
    }
    else if (addr < REG_MOVING)
    {
      // send servo position
      v = 0;
    }
    else
    {
      // v = userRead(addr);  
    }
    checksum += v;
    SerialUSB.write(v);
    addr++;
    bytes--;
  }

  return checksum;
}

void setup()
{
  Dxl.begin(1);
  SerialUSB.begin();

  bioloid.setup(MAX_NUM_SERVOS);
  pinMode(BOARD_LED_PIN, OUTPUT);
}

/*
 * Send status packet: FF FF id Len Err params=None checksum
 */
void statusPacket(int id, int err)
{
  SerialUSB.write(0xff);
  SerialUSB.write(0xff);
  SerialUSB.write(id);
  SerialUSB.write(2);
  SerialUSB.write(err);
  SerialUSB.write(255 - ((id + 2 + err) % 256));
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
  int i, poseSize;

  // process messages
  while (SerialUSB.available() > 0)
  {
    // We need two 0xFF at start of packet
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
          // return an error packet: FF FF id Len Err=bad checksum, params=None check
          statusPacket(id, ERR_CHECKSUM);
        }
        else if (id == 253) // ID = 253, ArbotiX instruction
        {
          // ID = 253, ArbotiX special instructions
          // Pose Size = 7, followed by single param: size of pose
          // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
          // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size)
          // Play Seq = A, no params
          switch (ins)
          {
            case ARB_WRITE_DATA:
              // send return packet
              blink(3); // Not implemented!!!
              //          statusPacket(id, handleWrite());
              break;

            case ARB_READ_DATA:
              checksum = id + params[1] + 2;
              SerialUSB.write(0xff);
              SerialUSB.write(0xff);
              SerialUSB.write(id);
              SerialUSB.write((unsigned char)2 + params[1]);
              SerialUSB.write((unsigned char)0);
              // send actual data
              checksum += handleRead();
              SerialUSB.write(255 - ((checksum) % 256));
              break;

            case ARB_SIZE_POSE: // Pose Size = 7, followed by single param: size of pose
              statusPacket(id, 0);
              bioloid.setPoseSize(params[0]);
              bioloid.readPose();
              //SerialUSB.println(poseSize);
              break;

            case ARB_LOAD_POSE: // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
              statusPacket(id, 0);
              poseSize = bioloid.getPoseSize();
//                                                      SerialUSB.print("New Pose:");
              for (i = 0; i < poseSize; i++)
              {
                poses[params[0]][i] = (params[(2 * i) + 1] + (params[(2 * i) + 2] << 8));
//                                                              SerialUSB.print(poses[params[0]][i]);
//                                                              SerialUSB.print(",");
              }
//                                                      SerialUSB.println("");
              break;

            case ARB_LOAD_SEQ: // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size) 
              statusPacket(id, 0);
              for (i = 0; i < (length - 2) / 3; i++)
              {
                sequence[i].pose = params[(i * 3)];
                sequence[i].time = params[(i * 3) + 1] + (params[(i * 3) + 2] << 8);
                //SerialUSB.print("New Transition:");
                //SerialUSB.print((int)sequence[i].pose);
                //SerialUSB.print(" in ");
                //SerialUSB.println(sequence[i].time);
              }
              break;

            case ARB_PLAY_SEQ: // Play Seq = A, no params   
              statusPacket(id, 0);
              seqPos = 0;
              while (sequence[seqPos].pose != 0xff)
              {
                // are we HALT?
                if (SerialUSB.available())
                {
                  if (SerialUSB.read() == 'H')
                    break;
                }

                int p = sequence[seqPos].pose;
                int poseSize = bioloid.getPoseSize();

                // load pose
                for (i = 0; i < poseSize; i++)
                {
                  bioloid.setNextPose(i + 1, poses[p][i]);
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
              break;

            case ARB_LOOP_SEQ: // Play Seq until we recieve a 'H'alt
              statusPacket(id, 0);
              while (1)
              {
                seqPos = 0;
                while (sequence[seqPos].pose != 0xff)
                {
                  // are we HALT?
                  if (SerialUSB.available())
                  {
                    if (SerialUSB.read() == 'H')
                      break;
                  }

                  int p = sequence[seqPos].pose;
                  poseSize = bioloid.getPoseSize();

                  // load pose
                  for (i = 0; i < poseSize; i++)
                  {
                    bioloid.setNextPose(i + 1, poses[p][i]);
                  }
                  // interpolate
                  bioloid.interpolateSetup(sequence[seqPos].time);
                  while (bioloid.interpolating())
                    bioloid.interpolateStep();
                  // next transition
                  seqPos++;
                }
              }
              break;

              // ARB_TEST is deprecated and removed

            default:
              blink(5); // Not implemented!!!  SEND ERROR  TODO
              break;
          }
        }
        else
        {
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
                for (i = 0; i < lennie; i++)
                {
                  SerialUSB.write(Dxl.getRxPacketParameter(i - 5));
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
              int x = DXL_MAKEWORD(params[1], params[2]);
              Dxl.writeWord(id, params[0], x);
            }
            // return a packet: FF FF id Len Err params check
            if (Dxl.getResult() == (1 << COMM_RXSUCCESS))
            {
              int lennie = Dxl.getRxPacketLength() + 4;
              if (lennie >= 6)
              {
                for (i = 0; i < lennie; i++)
                {
                  SerialUSB.write(Dxl.getRxPacketParameter(i - 5));
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

  // update joints
  bioloid.interpolateStep();
}

