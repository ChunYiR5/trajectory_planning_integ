#ifndef __SAVEDATA__
#define __SAVEDATA__

#include "AUORML/Utility/Header.h"

void SaveData_CreateFile( char *path ) ;
bool SaveData_OpenFile( const std::string& name );
void SaveData_Data(  double* Pos ,  double* Vel , double* Acc , double* PosCmd , double* VelCmd , double* AccCmd , double* ActTorq , double* CtrlTorq ) ;
void SaveData_CloseFile( );


#endif