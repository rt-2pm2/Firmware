/**
 *	Circular Array Library 
 */

#include "class_meas.hpp"
#include <stdio.h>
#include <stdlib.h>

// Initialize circ_arr
void init_circ_arr(struct circ_arr* ca, size_t dim)
{
	size_t i = 0;
	//pthread_mutex_init(&ca->mutex, 0);
	// Check the dimension
	if (dim > DIM_CA)
	{
		printf("Exceding the dimension of the array\n");
		return;
	}

	// Initialize the structure fields 
	ca->arr_dim = dim;
	ca->head = 0;
	ca->tail = 0;
	for (i = 0 ; i < DIM_CA ; i++)
		ca->ca[i] = 0;
	ca->N_values = 0;
}

// Check full circ_arr
int isfull(struct circ_arr* ca)
{
	int ret;
	
	//pthread_mutex_lock(&ca->mutex);
	if (ca->N_values == ca->arr_dim)
		ret = 1;
	else
		ret = 0;
	//pthread_mutex_unlock(&ca->mutex);
	return ret;
}

// Check empty circ_arr
int isempty(struct circ_arr* ca)
{
	int ret;

	//pthread_mutex_lock(&ca->mutex);
	if ( ca->N_values == 0 )
		ret = 1;
	else
		ret = 0;
	//pthread_mutex_unlock(&ca->mutex);
	return ret;
}

// Insert a value in the circ_arr 
size_t insert(struct circ_arr* ca, uint64_t val)
{
	size_t ret;

	// If not full update the number of inserted items
	if (!isfull(ca))
	{
		//pthread_mutex_lock(&ca->mutex);
		//printf("Inserting %d\n",val);
		ca->ca[ca->tail] = val;  
		ca->tail = (ca->tail + 1) % ca->arr_dim;
		ca->N_values++;	

		ret = ca->N_values;	
		//pthread_mutex_unlock(&ca->mutex);
			
		return ret;
	}
	else
	{
		puts("FULL!\n");
		return 0;
	}
}


// Retrieve a value from the circ_arr
uint64_t retrieve(struct circ_arr* ca, size_t i)
{
	size_t index = 0;
	uint64_t ret;	

	if (i > ca->arr_dim - 1)
	{
		printf("Index out of boundary!\n");
		return 0;
	}

	// Check for empty vector
	if (isempty(ca))
	{
		puts("Empty queue!");
		return 0;
	}

	index = (ca->head + i);
	if (index > ca->arr_dim - 1)
		index = index % ca->arr_dim;

	ret = ca->ca[index];

	return ret;
} 

// Retrieve a value from the circ_arr
int remove(struct circ_arr* ca)
{
	size_t index = 0;
	//uint_t ret = 0;

	// Check for empty vector
	if (isempty(ca))
	{
		return 0;
	}
	
	//ret = retrieve(ca, 0);
	ca->ca[ca->head] = 0;
	index = (ca->head + 1) % ca->arr_dim;
	ca->head = index;
	
	//pthread_mutex_lock(&ca->mutex);
	ca->N_values--;
	//pthread_mutex_unlock(&ca->mutex);

	//printf("[remove] Element = %lu\n", ret);
	return 1;
}

// Check the presence of an element in the queue
int ispresent(struct circ_arr* ca, uint64_t val)
{
	size_t N = 0;
	size_t i;
	int ret = -1;
	uint64_t queval;

	//pthread_mutex_lock(&ca->mutex);
	N = ca->N_values;	
	//pthread_mutex_unlock(&ca->mutex);

	for (i = 0; i < N; i++)
	{
		queval = retrieve(ca, i);	
		if ((queval == val) && (queval != 0))
		{
			ret = i;
		}
	}
	return ret;
}

// ===================================================
// ===================================================
//

MEASClass::MEASClass()
{
	// Initialize the circular array
	init_circ_arr(&tmstmp_circ_arr, 6);

	// Initialize the pin direction 
	gpio.setPinDir(PINOUT2, mmapGpio::OUTPUT);
}

MEASClass::~MEASClass()
{
}

void MEASClass::insert_control_timestamp(uint64_t t)
{
	pthread_mutex_lock(&(tmstmp_circ_arr.mutex));
	insert(&tmstmp_circ_arr, t);
	pthread_mutex_unlock(&(tmstmp_circ_arr.mutex));

	gpio.writePinHigh(PINOUT2);
}

int MEASClass::getindex_control_timestamp(uint64_t timestamp)
{
	int ret;
	pthread_mutex_lock(&(tmstmp_circ_arr.mutex));
	ret = ispresent(&tmstmp_circ_arr, timestamp);
	pthread_mutex_unlock(&(tmstmp_circ_arr.mutex));
	return ret;
}

void MEASClass::check_control_timestamp(uint64_t imutime)
{
	int indexfound = -1;
	int i;

	if (imutime != 0)	
	{
		pthread_mutex_lock(&(tmstmp_circ_arr.mutex));
		// New Method	
		indexfound = ispresent(&tmstmp_circ_arr, imutime);
		if (indexfound != -1)
		{
			// Put down the pin to signal the received packet
			gpio.writePinLow(PINOUT2);		

			// Remove the control timestamp from the queue
			for (i = 0; i < indexfound + 1; i++)
				remove(&tmstmp_circ_arr);
		}
		pthread_mutex_unlock(&(tmstmp_circ_arr.mutex));
	}
}



