# BT node type and status, from behaviortree_cpp/basic_types.h
uint8 UNDEFINED = 0
uint8 ACTION    = 1
uint8 CONDITION = 2
uint8 CONTROL   = 3
uint8 DECORATOR = 4
uint8 SUBTREE   = 5

uint8 IDLE      = 0
uint8 RUNNING   = 1
uint8 SUCCESS   = 2
uint8 FAILURE   = 3
uint8 SKIPPED   = 4

time stamp
string name
uint8 type
uint8 status
uint8 prev_status
