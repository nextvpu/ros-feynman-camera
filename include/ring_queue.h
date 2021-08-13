#ifndef __RING_QUEUE__
#define __RING_QUEUE__

#include <assert.h>
#include <string.h>

typedef unsigned char u_char;

#define CAN_WRITE 0x00
#define CAN_READ 0x01
#define READING 0x02
#define WRITING 0x03

typedef struct tag
{
	u_char tag_value;
}TAG;


class Ring_Queue
{
public:
	Ring_Queue(int nmemb, int size);
	~Ring_Queue();
	u_char * SOLO_Read();
	void SOLO_Read_Over();
	u_char * SOLO_Write();
	void SOLO_Write_Over();
private:
	u_char *queue_peek_nth(u_char *queue_p, int pos);
	u_char * _queue_p;
	int _nmemb;
	int _size;
	volatile int _read_now;
	volatile int _write_now;
};
#endif