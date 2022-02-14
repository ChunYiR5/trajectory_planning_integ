#include "AUORML/Utility/SaveData.h"
#include <stdio.h>

int counterOfSaveData = 0 ;
const int periodOfSaveData = 1 ; // 1ms
double recordTime = 0 ;

FILE *file ;

void SaveData_CreateFile( char *path )
{
	file=fopen( path , "w" ) ;
}

bool SaveData_OpenFile( const std::string& name )
{
	if (file = fopen(name.c_str(), "w")) {
        return true;
    } 
	else {
        return false;
    }   
}

void SaveData_Data( double* Pos ,  double* Vel , double* Acc , double* PosCmd , double* VelCmd , double* AccCmd , double* ActTorq , double* CtrlTorq )
{
	if( counterOfSaveData % periodOfSaveData == 0 )		
	{
		fprintf( file , "%f\t" , recordTime ) ;

		// Cartesian state & Cmd

		for( int i = 0 ; i < 6 ; i++ ) // 1
		{
			fprintf( file , "%f\t" , Pos[i] ) ;			
		}		

		for( int i = 0 ; i < 6 ; i++ ) // 1
		{
			fprintf( file , "%f\t" , Vel[i] ) ;			
		}

		for( int i = 0 ; i < 6 ; i++ ) // 1
		{
			fprintf( file , "%f\t" , Acc[i] ) ;			
		}		

		for( int i = 0 ; i < 6 ; i++ ) // 1
		{
			fprintf( file , "%f\t" , PosCmd[i] ) ;			
		}	
		
		for( int i = 0 ; i < 6 ; i++ ) // 1
		{
			fprintf( file , "%f\t" , VelCmd[i] ) ;			
		}		

		for( int i = 0 ; i < 6 ; i++ ) // 1
		{
			fprintf( file , "%f\t" , AccCmd[i] ) ;			
		}	
		
		for( int i = 0 ; i < 6 ; i++ ) // 1
		{
			fprintf( file , "%f\t" , ActTorq[i] ) ;			
		}	
		
		for( int i = 0 ; i < 6 ; i++ ) // 1
		{
			fprintf( file , "%f\t" , CtrlTorq[i] ) ;			
		}	
		
		fprintf( file , "\n" ) ;
	}
	
	counterOfSaveData++ ;
	recordTime += SAMPLINGTIME ;

}

void SaveData_CloseFile( )
{
	fclose( file ) ;
}