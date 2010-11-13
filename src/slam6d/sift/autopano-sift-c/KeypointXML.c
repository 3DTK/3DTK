
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* KeypointXML.cs
 *
 * Feature keypoint list XML input/output functionality.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 */

#include "AutoPanoSift.h"

#include <libxml/xmlreader.h>
#include <libxml/xmlwriter.h>

KeypointXMLList* KeypointXMLReader_ReadComplete (char* filename)
{
    return (KeypointXMLReader_ReadComplete2 (filename, (strcmp(filename+strlen(filename)-3, ".gz")==0)));
}

static char* String(const xmlChar* str) {
  char* value;
  value = strdup((char *)str);
  return value;
}

static int Int(const xmlChar* str) {
  int value;
  sscanf((char *)str, "%d", &value);
  return value;
}

static double Double(const xmlChar* str) {
  double value;
  sscanf((char *)str, "%lf", &value);
  return value;
}

KeypointXMLList* KeypointXMLReader_ReadComplete2 (char* filename, bool compressed)
{
  xmlTextReaderPtr reader;
  reader = xmlReaderForFile(filename, NULL, 0);

  KeypointXMLList* kl = NULL;

  if (reader != NULL) {
    int ret;
    const xmlChar *name;

    ret = xmlTextReaderRead(reader);
    while (ret == 1) {
	name = xmlTextReaderConstName(reader);
	if (xmlStrcmp( name, (xmlChar*) "KeypointXMLList")==0) {
	    kl = KeypointXMLList_new0();
	    ret = xmlTextReaderRead(reader);
	    while (ret == 1) {
		name = xmlTextReaderConstName(reader);
		if (xmlStrcmp(name, (xmlChar*) "XDim")==0 && xmlTextReaderNodeType(reader)==1) {
		    ret = xmlTextReaderRead(reader);
		    if (xmlTextReaderHasValue(reader))
			kl->xDim = Int(xmlTextReaderConstValue(reader));
		} else if (xmlStrcmp(name, (xmlChar*) "YDim")==0 && xmlTextReaderNodeType(reader)==1) {
		    ret = xmlTextReaderRead(reader);
		    if (xmlTextReaderHasValue(reader))
			kl->yDim = Int(xmlTextReaderConstValue(reader));
		} else if (xmlStrcmp(name, (xmlChar*) "ImageFile")==0 && xmlTextReaderNodeType(reader)==1) {
		    ret = xmlTextReaderRead(reader);
		    if (xmlTextReaderHasValue(reader) && xmlTextReaderNodeType(reader)==3)
			kl->imageFile = String(xmlTextReaderConstValue(reader));
		} else if (xmlStrcmp(name, (xmlChar*) "Arr")==0 && xmlTextReaderNodeType(reader)==1) {
		    ret = xmlTextReaderRead(reader);
		    while (ret == 1) {
			name = xmlTextReaderConstName(reader);
			if (xmlStrcmp(name, (xmlChar*) "KeypointN")==0 && xmlTextReaderNodeType(reader)==1) {
			    KeypointN* kp = KeypointN_new0();
			    ret = xmlTextReaderRead(reader);
			    while (ret == 1) {
				name = xmlTextReaderConstName(reader);
				if (xmlStrcmp(name, (xmlChar*) "X")==0 && xmlTextReaderNodeType(reader)==1) {
				    ret = xmlTextReaderRead(reader);
				    if (xmlTextReaderHasValue(reader))
					kp->x = Double(xmlTextReaderConstValue(reader));
				} else if (xmlStrcmp(name, (xmlChar*) "Y")==0 && xmlTextReaderNodeType(reader)==1) {
				    ret = xmlTextReaderRead(reader);
				    if (xmlTextReaderHasValue(reader))
					kp->y = Double(xmlTextReaderConstValue(reader));
				} else if (xmlStrcmp(name, (xmlChar*) "Scale")==0 && xmlTextReaderNodeType(reader)==1) {
				    ret = xmlTextReaderRead(reader);
				    if (xmlTextReaderHasValue(reader))
					kp->scale = Double(xmlTextReaderConstValue(reader));
				} else if (xmlStrcmp(name, (xmlChar*) "Orientation")==0 && xmlTextReaderNodeType(reader)==1) {
				    ret = xmlTextReaderRead(reader);
				    if (xmlTextReaderHasValue(reader))
					kp->orientation = Double(xmlTextReaderConstValue(reader));
				} else if (xmlStrcmp(name, (xmlChar*) "Dim")==0 && xmlTextReaderNodeType(reader)==1) {
				    ret = xmlTextReaderRead(reader);
				    if (xmlTextReaderHasValue(reader))
					kp->dim = Int(xmlTextReaderConstValue(reader));
				} else if (xmlStrcmp(name, (xmlChar*) "Descriptor")==0 && xmlTextReaderNodeType(reader)==1) {
				    KeypointN_CreateDescriptor(kp);
				    int i=0;
				    while (ret == 1) {
					name = xmlTextReaderConstName(reader);
					if (xmlStrcmp(name, (xmlChar*) "int")==0 && xmlTextReaderNodeType(reader)==1) {
					    if (xmlTextReaderNodeType(reader)==1) {
						ret = xmlTextReaderRead(reader);
						if (xmlTextReaderHasValue(reader))
						    kp->descriptor[i++] = Int(xmlTextReaderConstValue(reader));
					    }
					}
					if (xmlStrcmp(name, (xmlChar*) "Descriptor")==0 && xmlTextReaderNodeType(reader)==15) {
					    break;
					}
					ret = xmlTextReaderRead(reader);
				    }
				}
				if (xmlStrcmp(name, (xmlChar*) "Descriptor")==0 && xmlTextReaderNodeType(reader)==15) {
				    break;
				}
				ret = xmlTextReaderRead(reader);
			    }
			    KeypointXMLList_Add(kl, kp);
			    if (xmlStrcmp(name, (xmlChar*) "KeypointN")==0 && xmlTextReaderNodeType(reader)==15) {
				break;
			    }
			}
			ret = xmlTextReaderRead(reader);
		    }
		    if (xmlStrcmp(name, (xmlChar*)"Arr")==0 && xmlTextReaderNodeType(reader)==15) {
			break;
		    }
		}
		ret = xmlTextReaderRead(reader);
	    }
	    if (xmlStrcmp(name, (xmlChar*) "KeypointXMLList")==0 && xmlTextReaderNodeType(reader)==15) {
		break;
	    }
	}
	ret = xmlTextReaderRead(reader);
    }
    xmlFreeTextReader(reader);
  }
  xmlCleanupParser();
  
  return kl;
}


KeypointXMLList* KeypointXMLList_new0() 
{
    KeypointXMLList* self = (KeypointXMLList*)malloc(sizeof(KeypointXMLList));
    self->imageFile = NULL;
    self->array = ArrayList_new0(KeypointN_delete);
    return self;
}

void KeypointXMLList_delete(KeypointXMLList* self) 
{
    if (self->imageFile) {
	free(self->imageFile);
    }
    if (self->array) {
	ArrayList_delete(self->array);
    }
    free(self);
}

KeypointXMLList* KeypointXMLList_new (char* imageFile, int xDim, int yDim,
		     ArrayList* list)
{
    KeypointXMLList* self = KeypointXMLList_new0();
    self->imageFile = strdup(imageFile);
    self->array = ArrayList_new(ArrayList_Count(list), NULL);
    self->xDim = xDim;
    self->yDim = yDim;
    
    int n;
    for ( n = 0 ; n < ArrayList_Count(list) ; ++n)
	ArrayList_SetItem(self->array, n, ArrayList_GetItem(list, n));
    return self;
}

KeypointXMLList* KeypointXMLList_new2 (ArrayList* list, int xDim, int yDim)
{
    return KeypointXMLList_new("-memory-", xDim, yDim, list);
}

void KeypointXMLList_Add(KeypointXMLList* self, KeypointN* kp) 
{
  ArrayList_AddItem (self->array, kp);
}

void KeypointXMLWriter_WriteComplete (char* imageFile, int x, int y,
				      char* filename, ArrayList* list)
{
     KeypointXMLWriter_WriteComplete2 (imageFile, x, y, filename, list,
				      (strcmp(filename+strlen(filename)-3, ".gz")==0));
}

void KeypointXMLWriter_WriteComplete2 (char* imageFile, int x, int y,
				      char* filename, ArrayList* list, bool compressed)
{
	int rc;
	xmlTextWriterPtr writer;
	writer = xmlNewTextWriterFilename(filename, compressed);
	if (writer != NULL) {
		rc = xmlTextWriterSetIndent(writer, 1);
		rc = xmlTextWriterSetIndentString(writer, (xmlChar*) "  ");

		rc = xmlTextWriterStartDocument(writer, NULL, "utf-8", NULL);
		rc = xmlTextWriterStartElement(writer, (xmlChar*)"KeypointXMLList");

		rc = xmlTextWriterStartElement(writer, (xmlChar*)"XDim");
		rc = xmlTextWriterWriteFormatString(writer, "%d", x);
		rc = xmlTextWriterEndElement(writer);

		rc = xmlTextWriterStartElement(writer, (xmlChar*)"YDim");
		rc = xmlTextWriterWriteFormatString(writer, "%d", y);
		rc = xmlTextWriterEndElement(writer);

		rc = xmlTextWriterStartElement(writer, (xmlChar*)"ImageFile");
		rc = xmlTextWriterWriteString(writer, (xmlChar*)imageFile);
		rc = xmlTextWriterEndElement(writer);

		rc = xmlTextWriterStartElement(writer, (xmlChar*)"Arr");
		int n;
		for (n=0; n<ArrayList_Count(list); n++) {
			KeypointN* kp = (KeypointN*) ArrayList_GetItem(list, n);
			rc = xmlTextWriterStartElement(writer, (xmlChar*)"KeypointN");

			rc = xmlTextWriterStartElement(writer, (xmlChar*)"X");
			rc = xmlTextWriterWriteFormatString(writer, "%.15g", kp->x);
			rc = xmlTextWriterEndElement(writer);

			rc = xmlTextWriterStartElement(writer, (xmlChar*)"Y");
			rc = xmlTextWriterWriteFormatString(writer, "%.15g", kp->y);
			rc = xmlTextWriterEndElement(writer);

			rc = xmlTextWriterStartElement(writer, (xmlChar*)"Scale");
			rc = xmlTextWriterWriteFormatString(writer, "%.15g", kp->scale);
			rc = xmlTextWriterEndElement(writer);

			rc = xmlTextWriterStartElement(writer, (xmlChar*)"Orientation");
			rc = xmlTextWriterWriteFormatString(writer, "%.15g", kp->orientation);
			rc = xmlTextWriterEndElement(writer);

			rc = xmlTextWriterStartElement(writer, (xmlChar*)"Dim");
			rc = xmlTextWriterWriteFormatString(writer, "%d", kp->dim);
			rc = xmlTextWriterEndElement(writer);
			
			rc = xmlTextWriterStartElement(writer, (xmlChar*)"Descriptor");
			int i;
			for(i=0; i< kp->dim; i++) {
				rc = xmlTextWriterStartElement(writer, (xmlChar*)"int");
				rc = xmlTextWriterWriteFormatString(writer, "%d", kp->descriptor[i]);
				rc = xmlTextWriterEndElement(writer);
				
			}
			rc = xmlTextWriterEndElement(writer);

			rc = xmlTextWriterEndElement(writer);

		}

		rc = xmlTextWriterEndElement(writer);
		rc = xmlTextWriterEndDocument(writer);
		xmlFreeTextWriter(writer);		
	}
}


// Default constructor would not be allowed, as the keypoint can only be
// created from a floating point one. However, for the automatic
// serialization to work, we have to enable it.
KeypointN* KeypointN_new0 ()
{
  KeypointN* self = (KeypointN*)malloc(sizeof(KeypointN));
  self->domain.getDimensionCount = (int ( *)(IKDTreeDomain *)) KeypointN_GetDimensionCount;
  self->domain.getDimensionElement = (int ( *)(IKDTreeDomain *,int)) KeypointN_GetDimensionElement;
  self->dim = 0;
  self->descriptor = NULL;
  return self;
}

void KeypointN_delete (KeypointN* self)
{
  if (self->descriptor)
    free(self->descriptor);
  free(self);
}

void KeypointN_CreateDescriptor (KeypointN* self)
{
  if (self->dim != 0) 
    self->descriptor = (int*)malloc(self->dim * sizeof(int));  
}

KeypointN* KeypointN_clone (KeypointN* self)
{
  KeypointN* kc = KeypointN_new0 ();
  
  kc->x = self->x;
  kc->y = self->y;
  kc->scale = self->scale;
  kc->orientation = self->orientation;
  kc->dim = self->dim;
  if (self->dim) {
    KeypointN_CreateDescriptor (kc);
    memcpy(kc->descriptor, self->descriptor, self->dim*sizeof(int));
  }
  return (kc);
}


KeypointN* KeypointN_new (Keypoint* kp)
{
  if (kp->hasFV != true)
    FatalError("While trying to generate integer " 
	       "vector: source keypoint has no feature vector yet");

  KeypointN* self = KeypointN_new0 ();
  self->x = kp->x;
  self->y = kp->y;
  self->scale = kp->scale;
  self->orientation = kp->orientation;

  self->dim = Keypoint_FVLinearDim(kp);
  KeypointN_CreateDescriptor (self);
  int d;
  for (d = 0 ; d < Keypoint_FVLinearDim(kp) ; ++d) {
    self->descriptor[d] = (int) (255.0 * Keypoint_FVLinearGet (kp, d));
    if (self->descriptor[d] < 0 || self->descriptor[d] > 255) {
      FatalError("Resulting integer descriptor k is not 0 <= k <= 255 : %d", self->descriptor[d]);
    }
  }
  return self;
}

// IKDTreeDomain interface implementation
int KeypointN_GetDimensionCount(KeypointN* self) {
  return self->dim;
}

int KeypointN_GetDimensionElement (KeypointN* self, int n)
{
  return self->descriptor[n];
}
