#include "AutoPanoSift.h"

/* This is a replacement for the "GUIImage-Drawing.cs" */
/* the code is wrapping a libpano12 Image structure */

DisplayImage* DisplayImage_new0() {
	DisplayImage* self = (DisplayImage*)malloc(sizeof(DisplayImage));
	self->data = NULL;
	return self;
}

DisplayImage* DisplayImage_new(char* filename) {
	DisplayImage* self = DisplayImage_new0();
	fullPath filePath;
	if (StringtoFullPath(&filePath, filename) != 0) {
		FatalError("Syntax error: Not a valid pathname");
	}
#ifdef HAS_PANO13
	if ( panoImageRead(self, &filePath) == FALSE ) {
		FatalError("Syntax error: Not a valid image");
	}
#else
	if( readImage(self, &filePath) != 0) {
		FatalError("Syntax error: Not a valid image");
	}
#endif

	return self;
}

void DisplayImage_delete(DisplayImage* self) {
	if (self) {
	    if (self->data) {
		myfree((void**)self->data);
		self->data = NULL;
	    }
	    free(self);
	}
}

DisplayImage* DisplayImage_ScaleSimple(DisplayImage* self, int width, int height) 
{
	DisplayImage* im = DisplayImage_new0();
	memcpy(im, self, sizeof(DisplayImage));

	im->width = width;
	im->height = height;
	im->bytesPerLine = im->width * im->bitsPerPixel/8;
	im->dataSize = im->bytesPerLine * im->height;
	im->data = (unsigned char**) mymalloc( im->dataSize );
	if (im->data == NULL) {
		FatalError("out of memory!");
	}
	Image* src = self;
	Image* dest = im;


	/* modified copy from libpano12:filter.c:CopyImageData */

	register unsigned char 	*in, *out;
	register int 			x,y, id, is, i;
	int						bpp_s, bpp_d;
	double sx, sy;
	
	in 		= *(src->data);
	out 	= *(dest->data);
	sx		= (double)src->width / (double)dest->width;
	sy		= (double)src->height / (double)dest->height;
	bpp_s	= src->bitsPerPixel  / 8;
	bpp_d	= dest->bitsPerPixel / 8;
	
	for( y = 0; y < dest->height; y++)
	{
		for( x = 0; x < dest->width; x++)
		{
			is = (int)(y * sy) * src->bytesPerLine  + bpp_s * (int)(x * sx);
			id = y        * dest->bytesPerLine + bpp_d * x;

			if( (int)(y * sy) < 0 || (int)(y * sy) >= src->height ||
			    (int)(x * sx) < 0 || (int)(x * sx) >= src->width ) 	// outside src; set dest = 0
			{
				i = bpp_d;
				while( i-- > 0 ) out[ id++ ] = 0;
			}
			else 							// inside src; set dest = src
			{
				switch( bpp_d )
				{
				case 8: switch( bpp_s )
				{
				case 8:	memcpy( out + id, in + is, 8 ); 
					break;	
				case 6: out[id++] = 255U; out[id++] = 255U;
					memcpy( out + id, in + is, 6 ); 
					break;
				case 4: out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					break;
				case 3: out[id++] = 255U; out[id++] = 255U;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					break;
				}
					break;
				case 6:  switch( bpp_s )
				{
				case 8:	is += 2;
					memcpy( out + id, in + is, 6 ); 
					break;	
				case 6: memcpy( out + id, in + is, 6 ); 
					break;
				case 4: is++;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					break;
				case 3: out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					break;
				}
					break;

				case 4:  switch( bpp_s )
				{
				case 8:	out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					break;	
				case 6: out[id++] = 255U;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					break;
				case 4: memcpy( out + id, in + is, 4 ); 
					break;
				case 3: out[id++] = 255U;
					memcpy( out + id, in + is, 3 ); 
					break;
				}
					break;

				case 3:  switch( bpp_s )
				{
				case 8:	is+=2;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++; 
					break;	
				case 6: out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++; 
					break;
				case 4: is++;
					memcpy( out + id, in + is, 3 ); 
					break;
				case 3: memcpy( out + id, in + is, 3 ); 
					break;
				}
					break;

				}
			}
		}
	}

	return im;
}

double DisplayImage_ScaleWithin(DisplayImage* self, int dim) {

	if (self->width <= dim && self->height <= dim)
		return (1.0);
    
	double xScale = ((double) dim / self->width);
	double yScale = ((double) dim / self->height);
	double smallestScale = xScale <= yScale ? xScale : yScale;

	DisplayImage* scaled = DisplayImage_ScaleSimple (self, 
							 (int) (self->width * smallestScale+0.0001),
							 (int) (self->height * smallestScale+0.0001));

	if (self->data) {
		myfree((void**)self->data);
	}
	memmove(self, scaled, sizeof(DisplayImage));
	free(scaled);

	return (smallestScale);
}

DisplayImage* DisplayImage_Carve(DisplayImage* self, int dx, int dy, int width, int height)
{
	DisplayImage* result = DisplayImage_new0();
	memcpy(result, self, sizeof(DisplayImage));

	result->width = width;
	result->height = height;
	result->bytesPerLine = result->width * result->bitsPerPixel/8;
	result->dataSize = result->bytesPerLine * result->height;
	result->data = (unsigned char**) mymalloc( result->dataSize );
	if (result->data == NULL) {
		FatalError("out of memory!");
	}

	Image* src = self;
	Image* dest = result;

	/* modified copy from libpano12:filter.c:CopyImageData */
	register unsigned char 	*in, *out;
	register int 			x,y, id, is, i;
	int						bpp_s, bpp_d;
	
	in 		= *(src->data);
	out 	= *(dest->data);
	//dx		= (src->width  - dest->width)  / 2;
	//dy		= (src->height - dest->height) / 2;
	bpp_s	= src->bitsPerPixel  / 8;
	bpp_d	= dest->bitsPerPixel / 8;
	
	for( y = 0; y < dest->height; y++)
	{
		for( x = 0; x < dest->width; x++)
		{
			is = (y + dy) * src->bytesPerLine  + bpp_s * (x + dx);
			id = y        * dest->bytesPerLine + bpp_d * x;

			if( y + dy < 0 || y + dy >= src->height ||
			    x + dx < 0 || x + dx >= src->width ) 	// outside src; set dest = 0
			{
				i = bpp_d;
				while( i-- > 0 ) out[ id++ ] = 0;
			}
			else 						// inside src; set dest = src
			{
				switch( bpp_d )
				{
				case 8: switch( bpp_s )
				{
				case 8:	memcpy( out + id, in + is, 8 ); 
					break;	
				case 6: out[id++] = 255U; out[id++] = 255U;
					memcpy( out + id, in + is, 6 ); 
					break;
				case 4: out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					break;
				case 3: out[id++] = 255U; out[id++] = 255U;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					break;
				}
					break;
				case 6:  switch( bpp_s )
				{
				case 8:	is += 2;
					memcpy( out + id, in + is, 6 ); 
					break;	
				case 6: memcpy( out + id, in + is, 6 ); 
					break;
				case 4: is++;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					break;
				case 3: out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					out[id++] = in[ is++ ]; out[id++] = 0;
					break;
				}
					break;

				case 4:  switch( bpp_s )
				{
				case 8:	out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					break;	
				case 6: out[id++] = 255U;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					break;
				case 4: memcpy( out + id, in + is, 4 ); 
					break;
				case 3: out[id++] = 255U;
					memcpy( out + id, in + is, 3 ); 
					break;
				}
					break;

				case 3:  switch( bpp_s )
				{
				case 8:	is+=2;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++; 
					break;	
				case 6: out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++;
					out[id++] = in[ is++ ]; is++; 
					break;
				case 4: is++;
					memcpy( out + id, in + is, 3 ); 
					break;
				case 3: memcpy( out + id, in + is, 3 ); 
					break;
				}
					break;

				}
			}
		}
	}
	return result;
}

ImageMap* DisplayImage_ConvertToImageMap(DisplayImage* self)  //// dbl <= float
{
	ImageMap* result = ImageMap_new(self->width, self->height);
	if (self->bitsPerPixel == 32) {
	    LOOP_IMAGE(self,{
		    /* Data is (alpha,R,G,B) */
		    result->values[x][y] = (float)(((double)idata[1]+idata[2]+idata[3])/(255.0 * 3.0));
		});
	} else if (self->bitsPerPixel == 64) {
	    LOOP_IMAGE(self,{
		    unsigned short *udata=(unsigned short *)idata;
		    /* Data is (alpha,R,G,B) */
		    result->values[x][y] = (float)(((double)udata[1]+udata[2]+udata[3])/(65535 * 3.0));
		});
	} else {
	    /* It appears that TIFF library calls used (TIFFReadRGBA), will always return 32 or 64-bit, but just in case... */
	    FatalError("DisplayImage: Unsupported pixel size");
	}
	return result;
}
