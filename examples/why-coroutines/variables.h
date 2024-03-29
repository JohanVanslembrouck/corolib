/**
 * @file variables.h
 * @brief Global variables to be used by all examples.
 * Avoids declaring and initializing variables in the application code.
 * This will shorten the application code to make it more usable to use in documentation or slides.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _VARIABLES_H_
#define _VARIABLES_H_

int event1 = 1;
int event2 = 2;

int gret1 = 1, gin11 = 11, gin12 = 12, gout11 = 11, gout12 = 12;
int gval1 = 0;
int gret2 = 2, gin21 = 21, gin22 = 22, gout21 = 21;
int gret3 = 3, gin31 = 31, gout31 = 31, gout32 = 32;

int start_time;
int get_current_time() { return 0; }
int elapsed_time;

#endif
