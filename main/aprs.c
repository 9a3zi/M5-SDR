/* Bell 202 AFSK 1200 / AX.25 HDLC receiver, 48 kHz discriminator input.
 * Mark 1200 Hz, space 2200 Hz. NRZI, bit-stuffing, reflected CRC-16/X25.
 * Lightweight quadrature tone trackers, transition-aided symbol clock.
 * Decoder is experimental: verify with known on-air frames. */
#include "aprs.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define FS 48000.0f
#define BIT_SAMPLES 40.0f
#define MAX_BITS (340 * 8 + 32)
static const float c1=0.98768834f,s1=0.15643447f; /* 1200 Hz */
static const float c2=0.95881973f,s2=0.28401534f; /* 2200 Hz */
static float p1r,p1i,p2r,p2i, e1,e2, phase;
static int tone_prev=-1, nrzi_prev=-1;
static uint8_t shift8;
static uint8_t bits[MAX_BITS];
static int nbits;
static bool framed;
static uint32_t good,bad,revision,flags,transitions,afsk_samples;
static float signal_ema, quality_ema;
static aprs_packet_t history[APRS_HISTORY];
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

void aprs_reset(void) {
    p1r=p1i=p2r=p2i=e1=e2=phase=0;
    tone_prev=nrzi_prev=-1; shift8=0; nbits=0; framed=false;
    portENTER_CRITICAL(&lock);good=bad=flags=transitions=afsk_samples=0;signal_ema=quality_ema=0;revision++;memset(history,0,sizeof history);portEXIT_CRITICAL(&lock);
}
static void address(char *dst,size_t cap,const uint8_t *p) {
    char call[8];int n=0;
    for(int i=0;i<6;i++){char ch=(char)(p[i]>>1);if(ch!=' ' && n<6)call[n++]=ch;}
    call[n]=0;
    unsigned ss=(p[6]>>1)&15;
    if(ss)snprintf(dst,cap,"%s-%u",call,ss);
    else snprintf(dst,cap,"%s",call);
}
static void emit(const uint8_t *p,int len) {
    if(len<18)return;
    int off=0;char dst[16],src[16],line[APRS_LINE];
    address(dst,sizeof dst,p);address(src,sizeof src,p+7);
    off=14;
    char via[55]="";
    while(!(p[off-1]&1)){
        if(off+7>len-2 || off>14+7*7)return;
        char a[16];address(a,sizeof a,p+off);
        size_t n=strlen(via);
        snprintf(via+n,sizeof(via)-n,"%s%s",n?",":"",a);
        off+=7;
    }
    if(off+2>len-2)return;
    if(p[off]!=0x03 || p[off+1]!=0xF0)return; /* APRS UI frame */
    off+=2;
    int w=snprintf(line,sizeof line,"%s>%s%s%s: ",src,dst,via[0]?",":"",via);
    for(int i=off;i<len-2 && w<(int)sizeof(line)-1;i++){
        unsigned char c=p[i];
        line[w++]=(c>=32 && c<127)?(char)c:'.';
    }
    line[w]=0;
    portENTER_CRITICAL(&lock);
    for(int i=APRS_HISTORY-1;i>0;i--)history[i]=history[i-1];
    snprintf(history[0].text,sizeof history[0].text,"%s",line);
    good++;revision++;
    portEXIT_CRITICAL(&lock);
}
static void finish_frame(void) {
    if(nbits<16*8 || nbits>MAX_BITS-8)return;
    uint8_t data[340]={0};int count=0,ones=0;
    for(int i=0;i<nbits;i++){
        int bit=bits[i];
        if(bit){if(++ones>5)return;}
        else {if(ones==5){ones=0;continue;}ones=0;}
        if(count>=340*8)return;
        if(bit)data[count>>3]|=(uint8_t)(1u<<(count&7));
        count++;
    }
    if(count&7 || count<18*8)return;
    int len=count>>3;
    uint16_t crc=0xffff;
    for(int i=0;i<len;i++){
        crc^=data[i];
        for(int j=0;j<8;j++)crc=(crc&1)?(crc>>1)^0x8408:(crc>>1);
    }
    if(crc==0xf0b8)emit(data,len);
    else {portENTER_CRITICAL(&lock);bad++;revision++;portEXIT_CRITICAL(&lock);}
}
static void raw_bit(int b) {
    /* Flag is 01111110 in wire bit order, represented by shift register 0x7e. */
    shift8=(uint8_t)((shift8>>1)|(b?0x80:0));
    if(shift8==0x7e){
        portENTER_CRITICAL(&lock);flags++;portEXIT_CRITICAL(&lock);
        if(framed){nbits-=7;if(nbits>=0)finish_frame();}
        framed=true;nbits=0;return;
    }
    if(framed){if(nbits<MAX_BITS)bits[nbits++]=(uint8_t)b;else{framed=false;nbits=0;}}
}
static void symbol(int tone) {
    if(nrzi_prev>=0)raw_bit(tone==nrzi_prev?1:0);
    nrzi_prev=tone;
}
void aprs_feed(float x) {
    /* Complex IIR quadrature bandpass tone detectors, then smoothed energy. */
    const float a=0.075f;
    float r=p1r*c1-p1i*s1+a*x, i=p1r*s1+p1i*c1;
    p1r=r*(1-a);p1i=i*(1-a);
    r=p2r*c2-p2i*s2+a*x;i=p2r*s2+p2i*c2;
    p2r=r*(1-a);p2i=i*(1-a);
    float q1=p1r*p1r+p1i*p1i,q2=p2r*p2r+p2i*p2i;
    e1+=0.06f*(q1-e1);e2+=0.06f*(q2-e2);
    int tone=e1>=e2?0:1;
    /* Relative AFSK tone contrast; noise usually has much lower contrast.
       Diagnostics are deliberately not used as a hard gate yet. */
    float total=e1+e2;
    float contrast=total>1e-12f?fabsf(e1-e2)/total:0.0f;
    signal_ema+=0.0005f*(total-signal_ema);
    quality_ema+=0.0005f*(contrast-quality_ema);
    afsk_samples++;
    if(tone_prev>=0 && tone!=tone_prev){
        transitions++;
        /* Shift symbol timing toward transition boundary; avoid spurious jitter. */
        float err=phase<20.0f?phase:phase-40.0f;
        phase-=0.12f*err;
        if(phase<0)phase+=40.0f;
        if(phase>=40)phase-=40.0f;
    }
    tone_prev=tone;
    /* Transitions are bit boundaries (phase 0), but sample bits at the
       CENTER (phase 20), not at the next boundary (phase 40). */
    float old_phase=phase;
    phase+=1.0f;
    if(old_phase<BIT_SAMPLES*0.5f && phase>=BIT_SAMPLES*0.5f)symbol(tone);
    if(phase>=BIT_SAMPLES)phase-=BIT_SAMPLES;
}
uint32_t aprs_revision(void){uint32_t v;portENTER_CRITICAL(&lock);v=revision;portEXIT_CRITICAL(&lock);return v;}
uint32_t aprs_packet_count(void){uint32_t v;portENTER_CRITICAL(&lock);v=good;portEXIT_CRITICAL(&lock);return v;}
uint32_t aprs_crc_errors(void){uint32_t v;portENTER_CRITICAL(&lock);v=bad;portEXIT_CRITICAL(&lock);return v;}
int aprs_packets(aprs_packet_t *out,int cap){if(!out||cap<=0)return 0;if(cap>APRS_HISTORY)cap=APRS_HISTORY;
    portENTER_CRITICAL(&lock);int n=good<APRS_HISTORY?(int)good:APRS_HISTORY;if(n>cap)n=cap;memcpy(out,history,(size_t)n*sizeof(*out));portEXIT_CRITICAL(&lock);return n;}

uint32_t aprs_flags(void){uint32_t v;portENTER_CRITICAL(&lock);v=flags;portEXIT_CRITICAL(&lock);return v;}
uint32_t aprs_transitions(void){uint32_t v;portENTER_CRITICAL(&lock);v=transitions;portEXIT_CRITICAL(&lock);return v;}
int aprs_quality_pct(void){float q=quality_ema;int v=(int)(q*100.0f+0.5f);return v<0?0:v>100?100:v;}
