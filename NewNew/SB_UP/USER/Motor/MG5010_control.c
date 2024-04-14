#include "MG5010_control.h"

extern uint8_t can_send_data_5010[8];

void start_5010(void)
{
	can_send_data_5010[0] = 0x88;
	can_send_data_5010[1] = 0;
	can_send_data_5010[2] = 0;
	can_send_data_5010[3] = 0;
	can_send_data_5010[4] = 0;
	can_send_data_5010[5] = 0;
	can_send_data_5010[6] = 0;
	can_send_data_5010[7] = 0;
}

void speed_control_send(int32_t speedControl)
{
	can_send_data_5010[0] = 0xA2;
	can_send_data_5010[1] = 0;
	can_send_data_5010[2] = 0;
	can_send_data_5010[3] = 0;
	can_send_data_5010[4] = *(uint8_t *)(&speedControl);
	can_send_data_5010[5] = *((uint8_t *)(&speedControl)+1);
	can_send_data_5010[6] = *((uint8_t *)(&speedControl)+2);
	can_send_data_5010[7] = *((uint8_t *)(&speedControl)+3);
}
