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

#define MAX_NUM_SERVOS  5


#include <CM9_BC.h>

#include "opencm.h"

Dynamixel Dxl(1);
Dynamixel *pDxl = &Dxl;
BioloidController bioloid;

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
      v = digitalRead(addr - REG_DIGITAL_IN0);
    }
    else if (addr == REG_DIGITAL_IN1)
    {
      // digital 8-15
      v = digitalRead(addr - REG_DIGITAL_IN0);
    }
    else if (addr == REG_DIGITAL_IN2)
    {
      // digital 16-23
      v = digitalRead(addr - REG_DIGITAL_IN0);
    }
    else if (addr == REG_DIGITAL_IN3)
    {
      // digital 24-31
      v = digitalRead(addr - REG_DIGITAL_IN0);
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


/*
 * Handle a write to ArbotiX registers.
 */
int handleWrite()
{
  int addr = params[0];
  int bytes = params[1];
  int value;

  if (bytes == 1)
  {
    value = params[2];
  }
  else if (bytes == 2)
  {
    value = DXL_MAKEWORD(params[2], params[3]);
  }
  else
  {
    return ERR_WRONG_PARAMETER;
  }
  
// TODO: not tested!!!               REG_DIGITAL_OUT0    47  // First digital pin to write
//                                // base + index, bit 1 = value (0,1), bit 0 = direction (0,1)

  if (addr >= REG_DIGITAL_OUT0 && addr < REG_RESERVED)
  {
    digitalWrite(addr - REG_DIGITAL_OUT0, value);
  }

  return OK;
}

void setup()
{
  Dxl.begin(3);
  SerialUSB.begin();

  bioloid.setup(MAX_NUM_SERVOS);
  pinMode(BOARD_LED_PIN, OUTPUT);
  
  for (int id = 1; id <= MAX_NUM_SERVOS; id++)
    Dxl.jointMode(id);
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

void writeWord(int value, int &checksum)
{
  checksum += DXL_LOBYTE(value);
  checksum += DXL_HIBYTE(value);
  SerialUSB.write(DXL_LOBYTE(value));
  SerialUSB.write(DXL_HIBYTE(value));
}


/*
 * packet: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix
 *
 * ID = 253 for these special commands!
 * Read Pose  = 4, read position for the first N servos  XXX experimental
 * Write Pose = 5, write position for the first N servos  XXX experimental
 * Pose Size  = 7, followed by single param: size of pose
 * Load Pose  = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
 * Seq Size   = 9, followed by single param: size of seq
 * Load Seq   = A, followed by index/times (# of parameters = 3*seq_size) 
 * Play Seq   = B, no params
 * Loop Seq   = C, 
 */
void loop()
{
  int i, poseSize;
  bool error = false;

  // process messages
  while (SerialUSB.available() > 0)
  {
    // We need two 0xFF at start of packet
    if (mode == 0) // start of new packet
    {
      if (SerialUSB.read() == 0xff)
      {
        mode = 1;
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
          statusPacket(id, ERR_WRONG_CHECKSUM);
        }
        else if (id == 253) // ID = 253, ArbotiX instruction
        {
          // ID = 253, ArbotiX special instructions
          // Read Pose  = 4, read position for the first N servos  XXX experimental
          // Read P/V/E = 5, read position, velocity and effort for the first N servos  XXX experimental
          // Write Pose = 6, write position for the first N servos  XXX experimental
          // Pose Size  = 7, followed by single param: size of pose
          // Load Pose  = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
          // Load Seq   = 9, followed by index/times (# of parameters = 3*seq_size)
          // Play Seq   = A, no params
          switch (ins)
          {
            case ARB_WRITE_DATA:
              statusPacket(id, handleWrite());
              break;

            case ARB_READ_DATA:
              checksum = id + params[1] + 2;
              SerialUSB.write(0xff);
              SerialUSB.write(0xff);
              SerialUSB.write(id);
              SerialUSB.write((unsigned char)2 + params[1]);
              SerialUSB.write((unsigned char)OK);
              // send actual data
              checksum += handleRead();
              SerialUSB.write(255 - ((checksum) % 256));
              break;

            case ARB_READ_POSE:
              if (params[1] > MAX_NUM_SERVOS)
              {
                // return an error packet: FF FF id Len Err=wrong parameter, params=None check
                statusPacket(id, ERR_WRONG_PARAMETER);
                break;
              }
              
              // read present position for servos from ID = 1 to ID = params[1]
              checksum = id + 2 + 2*params[1];
              SerialUSB.write(0xff);
              SerialUSB.write(0xff);
              SerialUSB.write(id);
              SerialUSB.write((unsigned char)2 + 2*params[1]);  // id + err + 2 * <servos to read>
              SerialUSB.write((unsigned char)OK);  // we report always OK, but wrong positions will be -1
              for (int servo_id = 1; servo_id <= params[1]; servo_id++)
              {
                writeWord(Dxl.getPosition(servo_id), checksum);
              }
              SerialUSB.write(255 - ((checksum) % 256));
              break;

            case ARB_READ_P_E:
              if (params[1] > MAX_NUM_SERVOS)
              {
                // return an error packet: FF FF id Len Err=wrong parameter, params=None check
                statusPacket(id, ERR_WRONG_PARAMETER);
                break;
              }
              
              // read present position and effort for servos from ID = 1 to ID = params[1]
              checksum = id + 2 + 4*params[1];
              SerialUSB.write(0xff);
              SerialUSB.write(0xff);
              SerialUSB.write(id);
              SerialUSB.write((unsigned char)2 + 4*params[1]);  // id + err + 4 * <servos to read>
              SerialUSB.write((unsigned char)OK);  // we report always OK, but wrong values will be -1
              for (int servo_id = 1; servo_id <= params[1]; servo_id++)
              {
                writeWord(Dxl.getPosition(servo_id), checksum);
                writeWord(Dxl.getLoad(servo_id), checksum);
              }
              SerialUSB.write(255 - ((checksum) % 256));
              break;

            case ARB_READ_P_V_E:
              if (params[1] > MAX_NUM_SERVOS)
              {
                // return an error packet: FF FF id Len Err=wrong parameter, params=None check
                statusPacket(id, ERR_WRONG_PARAMETER);
                break;
              }
              
              // read present position, velocity and effort for servos from ID = 1 to ID = params[1]
              checksum = id + 2 + 6*params[1];
              SerialUSB.write(0xff);
              SerialUSB.write(0xff);
              SerialUSB.write(id);
              SerialUSB.write((unsigned char)2 + 6*params[1]);  // id + err + 6 * <servos to read>
              SerialUSB.write((unsigned char)OK);  // we report always OK, but wrong values will be -1
              for (int servo_id = 1; servo_id <= params[1]; servo_id++)
              {
                writeWord(Dxl.getPosition(servo_id), checksum);
                writeWord(Dxl.getSpeed(servo_id), checksum);
                writeWord(Dxl.getLoad(servo_id), checksum);
              }
              SerialUSB.write(255 - ((checksum) % 256));
              break;

            case ARB_WRITE_POSE:
              if (params[1] > MAX_NUM_SERVOS)
              {
                // return an error packet: FF FF id Len Err=wrong parameter, params=None check
                statusPacket(id, ERR_WRONG_PARAMETER);
                break;
              }

              // write position for servos from ID = 1 to ID = params[1]; ignore negative values
              for (int servo_id = 1; servo_id <= params[1]; servo_id++)
              {
                word pos = DXL_MAKEWORD(params[servo_id*2], params[servo_id*2 + 1]);
                if (pos >= 0)
                  Dxl.goalPosition(servo_id, pos);
              }
              statusPacket(id, OK);
              break;

            case ARB_SIZE_POSE: // Pose Size = 7, followed by single param: size of pose
              statusPacket(id, OK);
              bioloid.setPoseSize(params[0]);
              bioloid.readPose();
              //SerialUSB.println(poseSize);
              break;

            case ARB_LOAD_POSE: // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
              statusPacket(id, OK);
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
              statusPacket(id, OK);
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
              statusPacket(id, OK);
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
              statusPacket(id, OK);
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
              statusPacket(id, ERR_NOT_IMPLEMENTED);
              blink(3); // Not implemented!!!  SEND ERROR  TODO
              break;
          }
        }
        else
        {
          // pass thru
          if (ins == INST_READ)
          {
            if (params[1] == 1)
            {
              byte value = Dxl.readByte(id, params[0]);
              if (value == 0xFF)
              {
                statusPacket(id, Dxl.getResult());
              }
              else
              {
                checksum = id + 2 + 1;
                SerialUSB.write(0xff);
                SerialUSB.write(0xff);
                SerialUSB.write(id);
                SerialUSB.write((unsigned char)2 + 1); // id + err + 1 data byte
                SerialUSB.write((unsigned char)OK);
                checksum += value;
                SerialUSB.write(value);
                SerialUSB.write(255 - ((checksum) % 256));
              }
            }
            else if (params[1] == 2)
            {
              word value = Dxl.readWord(id, params[0]);
              if (value == 0xFFFF)
              {
                statusPacket(id, Dxl.getResult());
              }
              else
              {
                checksum = id + 2 + 2;
                SerialUSB.write(0xff);
                SerialUSB.write(0xff);
                SerialUSB.write(id);
                SerialUSB.write((unsigned char)2 + 2); // id + err + 2 data bytes
                SerialUSB.write((unsigned char)OK);
                checksum += DXL_LOBYTE(value);
                checksum += DXL_HIBYTE(value);
                SerialUSB.write(DXL_LOBYTE(value));
                SerialUSB.write(DXL_HIBYTE(value));
                SerialUSB.write(255 - ((checksum) % 256));
              }
            }
            else if (params[1] == 4)
            {
              uint32 value = Dxl.readDword(id, params[0]);
              if (value == 0xFFFFFFFF)
              {
                statusPacket(id, Dxl.getResult());
              }
              else
              {
                checksum = id + 2 + 4;
                SerialUSB.write(0xff);
                SerialUSB.write(0xff);
                SerialUSB.write(id);
                SerialUSB.write((unsigned char)2 + 4); // id + err + 4 data bytes
                SerialUSB.write((unsigned char)OK);
                checksum += DXL_LOBYTE(DXL_LOWORD(value));
                checksum += DXL_HIBYTE(DXL_LOWORD(value));
                checksum += DXL_LOBYTE(DXL_HIWORD(value));
                checksum += DXL_HIBYTE(DXL_HIWORD(value));
                SerialUSB.write(DXL_LOBYTE(DXL_LOWORD(value)));
                SerialUSB.write(DXL_HIBYTE(DXL_LOWORD(value)));
                SerialUSB.write(DXL_LOBYTE(DXL_HIWORD(value)));
                SerialUSB.write(DXL_HIBYTE(DXL_HIWORD(value)));
                SerialUSB.write(255 - ((checksum) % 256));
              }
            }
            else
            {
              statusPacket(id, ERR_WRONG_PARAMETER);
            }
          }
          else if (ins == INST_WRITE)
          {
            if (length == 4)
            {
              statusPacket(id, Dxl.writeByte(id, params[0], params[1])
                        ? OK : Dxl.getResult());
            }
            else if (length == 5)
            {
              statusPacket(id, Dxl.writeWord(id, params[0], DXL_MAKEWORD(params[1], params[2]))
                        ? OK : Dxl.getResult());
            }
            else if (length == 7)
            {
              statusPacket(id, Dxl.writeDword(id, params[0], DXL_MAKEDWORD(DXL_MAKEWORD(params[1], params[2]),
                                                                           DXL_MAKEWORD(params[3], params[4])))
                        ? OK : Dxl.getResult());
            }
            else
            {
              statusPacket(id, ERR_WRONG_PARAMETER);
            }
          }
        }
      }
    }
  }

  // update joints
  bioloid.interpolateStep();
}
