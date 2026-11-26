#include "protocol.h"
#include "avi.h"
#include "timebase.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static unsigned rng=0x439823;
static unsigned random_byte(void){rng=rng*1664525u+1013904223u;return rng>>24;}
int main(int argc,char **argv){
    assert(!dart_is_stale(1000,1001,1000000));
    assert(!dart_is_stale(50,0xFFFFFFF0u,1000000));
    assert(dart_is_stale(2000001,1000000,1000000));
    for(unsigned period=72000;period<=384000;period+=312000){
        assert(dart_systick_micros(123,period-1,period-1)==123000);
        assert(dart_systick_micros(123,period-1,period/2-1)==123500);
        uint32_t before=dart_systick_micros(123,period-1,0);
        uint32_t after=dart_systick_micros(124,period-1,period-1);
        assert(before==123999&&after-before==1);
    }
    assert(dart_crc16("123456789",9)==0x29B1);
    for(unsigned run=0;run<10000;run++){
        uint8_t payload[80],wire[96];unsigned n=run%81;
        for(unsigned i=0;i<n;i++)payload[i]=(uint8_t)(run%3==0?0:random_byte());
        size_t count=dart_encode(16,run,payload,n,wire);assert(count<=96&&wire[count-1]==0);
        dart_parser parser={0};dart_packet out;int result=0;
        for(unsigned i=0;i<count;i++)result=dart_parse(&parser,wire[i],&out);
        assert(result==1&&out.seq==run&&out.type==16&&out.length==n&&!memcmp(out.data,payload,n));
        if(n){wire[count/2]^=1;int accepted=0;for(unsigned i=0;i<count;i++)if(dart_parse(&parser,wire[i],&out)==1)accepted++;assert(!accepted);}
        count=dart_encode(2,99,"STATUS",6,wire);
        for(unsigned i=0;i<count;i++)result=dart_parse(&parser,wire[i],&out);
        assert(result==1&&out.seq==99);
    }
    dart_parser parser={0};dart_packet out;for(unsigned i=0;i<200;i++)dart_parse(&parser,1,&out);assert(dart_parse(&parser,0,&out)==-1);
    uint8_t wire[96];size_t n=dart_encode(1,3,"X",1,wire);int result=0;for(unsigned i=0;i<n;i++)result=dart_parse(&parser,wire[i],&out);assert(result==1);
    uint8_t h[224];dart_avi_header(h,3,60000,1000,600,100,320,240);
    assert(!memcmp(h,"RIFF",4)&&dart_get32(h+64)==320&&dart_get32(h+68)==240&&dart_get32(h+48)==3);
    assert(h[160]==64&&h[161]==1&&h[162]==240&&h[163]==0);
    if(argc==2){FILE *file=fopen(argv[1],"wb");assert(file);fwrite(h,1,sizeof(h),file);fclose(file);}
    puts("PASS: 10000 randomized packet round trips, corruption rejection, stream resync, overflow recovery, AVI header");
    return 0;
}
