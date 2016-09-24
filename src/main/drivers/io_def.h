#pragma once

#include "common/utils.h"



#define DEFIO_TAG(pinid) CONCAT(DEFIO_TAG__, pinid)
#define DEFIO_TAG__NONE 0




#define DEFIO_REC(pinid) CONCAT(DEFIO_REC__, pinid)
#define DEFIO_REC__NONE NULL

#define DEFIO_IO(pinid) (IO_t)DEFIO_REC(pinid)



#define DEFIO_REC_INDEXED(idx) (ioRecs + (idx))


#define DEFIO_TAG_MAKE(gpioid, pin) ((((gpioid) + 1) << 4) | (pin))
#define DEFIO_TAG_ISEMPTY(tag) (!(tag))
#define DEFIO_TAG_GPIOID(tag) (((tag) >> 4) - 1)
#define DEFIO_TAG_PIN(tag) ((tag) & 0x0f)


#include "target.h"

#include "io_def_generated.h"

