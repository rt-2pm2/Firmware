/*
 *  Library for the circular array
 */
#ifndef CIRC_ARR_H
#define CIRC_ARR_H

#define DIM_CA 10 

#include <pthread.h>
#include <drivers/mmapGpio/mmapGpio.h>

#include <cstdint>
#include <stddef.h>

// Define the pins number
#define PINOUT1 (17)
#define PINOUT2 (5)
#define PINOUT3 (6)



struct circ_arr 
{
	uint64_t ca[DIM_CA];
	size_t head;	
	size_t tail;
	size_t N_values;
	size_t arr_dim;

	pthread_mutex_t mutex;
};

// --- Functions ---    
void init_circ_arr(struct circ_arr* ca, size_t dim);
int isfull(struct circ_arr* ca);
int isempty(struct circ_arr* ca);
size_t insert(struct circ_arr* ca, uint64_t val);
uint64_t retrieve(struct circ_arr* ca, size_t i);
int remove(struct circ_arr* ca);
int ispresent(struct circ_arr* ca, uint64_t val);

// ----- Class -----
class MEASClass
{
	public:
		MEASClass();
		~MEASClass();

		/* Set the last control timestamp */
		void insert_control_timestamp(uint64_t t);
		int getindex_control_timestamp(uint64_t t);
		void check_control_timestamp(uint64_t);

	private:
		struct circ_arr tmstmp_circ_arr;
		mmapGpio gpio; // Structure to control GPIO
};	
#endif

