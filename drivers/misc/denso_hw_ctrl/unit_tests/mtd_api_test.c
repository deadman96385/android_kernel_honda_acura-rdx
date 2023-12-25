
#include "mtd_api.h"

int main()
{

  char write_data[8] = "DEADBEEF";
  char read_data[8];

  /* PRIMARY TEST */
  WriteNor( PRIMARY, 0x00, write_data, sizeof(write_data) );
  ReadNor( PRIMARY, 0x00, read_data, sizeof(read_data) );

  if( strcmp( read_data, write_data, sizeof(read_data) ) != 0 )
  {
    return 1;
  }

  /* SECONDARY TEST */
  WriteNor( SECONDARY, 0x00, write_data, sizeof(write_data) );
  ReadNor( SECONDARY, 0x00, read_data, sizeof(read_data) );

  if( strcmp( read_data, write_data, sizeof(read_data) ) != 0 )
  {
    return 1;
  } 

  /* READONLY TEST */
  WriteNor( READONLY, 0x00, write_data, sizeof(write_data) );
  ReadNor( READONLY, 0x00, read_data, sizeof(read_data) );

  if( strcmp( read_data, write_data, sizeof(read_data) ) != 0 )
  {
    return 1;
  } 

  return 0;
}
