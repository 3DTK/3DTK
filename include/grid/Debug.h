#ifndef __DEBUG_H_
#define __DEBUG_H_

#include <iostream>

/** Printouts for debugging
 * @author Andre Schemschatt, Uwe Hebbelmann, Sebastian Stock
 * @date 14.2.08
 */

#ifdef DEBUG

#define DEBUGPRINT(msg) std::cout <<"[Debug] "<< msg << std::endl;
#else
#define DEBUGPRINT(msg) 
#endif

#endif
