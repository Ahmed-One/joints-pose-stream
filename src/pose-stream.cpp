/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */

#include "pose-stream.hpp"
#include "drive.h"

Drive drive;
RingBuffer<BUFF_SIZE> buffer;

