/*  GIMP header image file format (RGB): /home/jesus/Imágenes/frogger/morado2.h  */

static unsigned int width = 20;
static unsigned int height = 20;

/*  Call this macro repeatedly.  After each use, the pixel data can be extracted  */

#define HEADER_PIXEL(data,pixel) {\
pixel[0] = (((data[0] - 33) << 2) | ((data[1] - 33) >> 4)); \
pixel[1] = ((((data[1] - 33) & 0xF) << 4) | ((data[2] - 33) >> 2)); \
pixel[2] = ((((data[2] - 33) & 0x3) << 6) | ((data[3] - 33))); \
data += 4; \
}
static char *header_data =
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FQ$X!!!!!!!!FQ$XFQ$X"
	"!!!!!!!!FQ$X!!!!!!!!FQ$XFQ$X!!!!!!!!!!!!!!!!!!!!!!!!!!!!FQ$XFQ$X"
	"FQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!FQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!!!!!"
	"!!!!!!!!FQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$X"
	"FQ$XFQ$X!!!!!!!!!!!!!!!!FQ$X!!$X`Q!!!!$XFQ$XFQ$X!!$X`Q!!!!$XFQ$X"
	"FQ$XFQ$X!!$X`Q!!!!$XFQ$X!!!!!!!!!!!!!!!!FQ$XFQ$X!!$XFQ$XFQ$XFQ$X"
	"FQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$X!!!!!!!!!!!!!!!!FQ$XFQ$X"
	"FQ$XFQ$XFQ$XFQ$XFQ$XFQ$X`Q!!FQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!!!!!"
	"!!!!!!!!FQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$X`Q!!!!$X`Q!!!!$XFQ$XFQ$X"
	"FQ$XFQ$X!!!!!!!!!!!!!!!!FQ$XFQ$X!!$X`Q!!!!$XFQ$XFQ$X!!$X`Q!!FQ$X"
	"!!$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!!!!!!!!!!!!!FQ$XFQ$XFQ$X!!$XFQ$XFQ$X"
	"!!$X`Q!!!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!!!!!!!!!!!!!FQ$XFQ$X"
	"FQ$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$X!!!!!!!!"
	"!!!!!!!!FQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X`Q!!FQ$XFQ$X!!$X`Q!!!!$X"
	"FQ$XFQ$X!!!!!!!!!!!!!!!!!!$X`Q!!!!$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!FQ$X"
	"FQ$XFQ$X!!$XFQ$XFQ$XFQ$X!!!!!!!!!!!!!!!!FQ$X!!$XFQ$XFQ$X!!$XFQ$X"
	"FQ$X!!$X`Q!!!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!!!!!!!!!!!!!FQ$XFQ$X"
	"FQ$X!!$X`Q!!!!$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X!!!!!!!!"
	"!!!!!!!!FQ$XFQ$XFQ$XFQ$X!!$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$XFQ$X"
	"FQ$XFQ$X!!!!!!!!!!!!!!!!FQ$XFQ$X!!$XFQ$XFQ$X!!!!!!!!FQ$X!!!!!!!!"
	"FQ$X!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	"";
