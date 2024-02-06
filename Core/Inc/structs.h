#ifndef STRUCTSH
#define STRUCTSH
/*
struct  mc_data{
        struct integer_vector coordinate;
        uint32_t speed;
        uint8_t motor_direction;
        //the first 16 bits of the BSRR register sets the particular bit,
        //the least 16 bits of the BSRR register clear the particular bit
        //bit012 set pin (positive axis direction) bit567 clear pin (negative axis direction)
        //has to be shifted into the particular bits of the BSRR register for axis direction
        //PORTE PIN789 is xyz
        uint8_t motor_start;    //bit0=xstart//bit1=ystart//bit2=zstart
        uint8_t info_flags_1;         //bit0=last struct (contains already no information anymore)
        uint8_t info_flags_2;
        uint32_t line_number;

        //NEW!!

        int32_t Baxis
        int32_t Caxis

};
*/
//everything devided into 8byte segments because of can1 fifo size
//this is the same struct like mc_data only other partition for fast assignment
struct axis_data{
	int32_t step_count;
	uint32_t glass_scale_count;
};


#endif
