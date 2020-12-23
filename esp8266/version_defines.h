/*

   Versioning for protocol, device, etc.
 
*/


#define  FLOATHUB_PROTOCOL_VERSION 2
#define  FLOATHUB_ENCRYPT_VERSION  2


//
//	Debugging options
//

//#define HTTP_DEBUG_ON
//#define MDNS_DEBUG_ON
//#define STAT_DEBUG_ON
//#define INPT_DEBUG_ON
//#define WIFI_DEBUG_ON
//#define FILE_DEBUG_ON
//#define FILE_SERVE_ON	// Useful when debuggig to see SPIFF files from a browser
//#define CELL_DEBUG_ON
//#define AISR_DEBUG_ON


//
//  Global defines
//

#define MAX_COOKIES 10
// #define CELLULAR_CODE_ON	


#ifdef CELLULAR_CODE_ON
#define  FLOATHUB_MODEL_DESCRIPTION "FHC.P9.N3.v2.3.0"
#else
#define  FLOATHUB_MODEL_DESCRIPTION "FHW.P9.N3.v2.3.0"
#endif

//
//  Have to have a separate string for cellular debugging as cellular stuff
// is called on an interrupt so we can'd directly print stuff out
// (especially over virtual serial) during the interrupt call
//
#ifdef CELL_DEBUG_ON
String cellular_debug_string;
#endif
