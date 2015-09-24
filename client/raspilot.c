/* 
 * Copyright (c) 2015, Netforce Co. Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <lua5.1/lua.h>
#include <lua5.1/lauxlib.h>
#include <lua5.1/lualib.h>
#include <stdlib.h>
#include <string.h>
#include "libavcodec/avcodec.h"
#include <libswscale/swscale.h>
#include <math.h>

lua_State *L;

char screen_text[1024];
uint8_t *picture_data;
int picture_w, picture_h;

AVCodec *codec;
AVCodecContext *codec_ctx;
AVFrame *picture;
int got_frame=0;

SDL_Window *window;
SDL_Renderer *renderer;
SDL_Texture *yuv;
TTF_Font *font;

double ins_imu_rxz,ins_imu_ryz;
double ins_target_rxz,ins_target_ryz;
double ins_comp_x,ins_comp_y;
double ins_alt;
double ins_target_alt;
double ins_bat_volt;
int ins_throttle;

double orig_alt=0;

int prev_timer_t=0;

void on_timer() {
    int res;
    lua_getglobal(L,"on_timer");
    res=lua_pcall(L,0,0,0);
    if (res!=0) {
        error(L,"lua error: %s",lua_tostring(L,-1));
    }
}

void on_key(unsigned char key, int x, int y) {
    int res;
    lua_getglobal(L,"on_key");
    lua_pushlstring(L,&key,1);
    lua_pushnumber(L,x);
    lua_pushnumber(L,y);
    res=lua_pcall(L,3,0,0);
    if (res!=0) {
        error(L,"lua error: %s",lua_tostring(L,-1));
    }
}

int joy_x1=0, joy_y1=0, joy_x2=0, joy_y2=0; 
int JOY_MIN_INTERVAL=100;
int JOY_MAX_INTERVAL=1000;
int prev_joy_axis_change_t=0;
int joy_axis_changed=0;

void on_joy_axis() {
    int res;
    lua_getglobal(L,"on_joy_axis");
    lua_pushnumber(L,joy_x1);
    lua_pushnumber(L,joy_y1);
    lua_pushnumber(L,joy_x2);
    lua_pushnumber(L,joy_y2);
    res=lua_pcall(L,4,0,0);
    if (res!=0) {
        error(L,"lua error: %s",lua_tostring(L,-1));
    }
}

void on_joy_button(int button) {
    int res;
    lua_getglobal(L,"on_joy_button");
    lua_pushnumber(L,button);
    res=lua_pcall(L,1,0,0);
    if (res!=0) {
        error(L,"lua error: %s",lua_tostring(L,-1));
    }
}

void on_joy_hat(int button) {
    int res;
    lua_getglobal(L,"on_joy_hat");
    lua_pushnumber(L,button);
    res=lua_pcall(L,1,0,0);
    if (res!=0) {
        error(L,"lua error: %s",lua_tostring(L,-1));
    }
}

void render_screen() {
    renderer=SDL_CreateRenderer(window,-1,0);
    if (!renderer) {
        printf("failed to create renderer\n");
        exit(1);
    }
    if (got_frame) { // XXX
        SDL_SetRenderDrawColor(renderer,0,0,0,255);
        SDL_RenderFillRect(renderer,NULL);
    }

    SDL_RenderSetLogicalSize(renderer,640,480);

    if (got_frame) {
        yuv=SDL_CreateTexture(renderer,SDL_PIXELFORMAT_YV12,SDL_TEXTUREACCESS_STATIC,320,240);
        if (!yuv) {
            printf("failed to create texture\n");
            exit(1);
        }
        SDL_UpdateYUVTexture(yuv,NULL,picture->data[0],picture->linesize[0],picture->data[1],picture->linesize[1],picture->data[2],picture->linesize[2]);
        SDL_RenderCopy(renderer,yuv,NULL,NULL);
        SDL_DestroyTexture(yuv);
    }

    SDL_Color fg_color={0,255,0};
    SDL_Surface *font_surface=TTF_RenderText_Blended(font,screen_text,fg_color);
    SDL_Texture *font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    int font_w, font_h; 
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect={10,480-font_h-10,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    hlineColor(renderer,300,340,240,0xff00ff00);
    vlineColor(renderer,320,230,250,0xff00ff00);

    // draw heading
    double H=atan2(ins_comp_y,ins_comp_x)*180/M_PI;
    aacircleColor(renderer,500,60,50,0xff00ff00);
    vlineColor(renderer,500,60,10,0xff00ff00);
    int x1,y1,x2,y2;
    x1=500;
    y1=60;
    x2=500+50*sin(-H*M_PI/180);
    y2=60-50*cos(-H*M_PI/180);
    thickLineColor(renderer,x1,y1,x2,y2,3,0xff00ff00);

    // draw pitch/roll
    double P=asin(ins_imu_rxz)*180.0/M_PI;
    double R=asin(ins_imu_ryz)*180.0/M_PI;
    double TP=asin(ins_target_rxz)*180.0/M_PI;
    double TR=asin(ins_target_ryz)*180.0/M_PI;
    x1=320-240.0*cos(R*M_PI/180);
    y1=240+240.0*sin(R*M_PI/180)+500.0*sin(P*M_PI/180);
    x2=320+240.0*cos(R*M_PI/180);
    y2=240-240.0*sin(R*M_PI/180)+500.0*sin(P*M_PI/180);
    aalineColor(renderer,x1,y1,x2,y2,0xff00ff00);
    aalineColor(renderer,x1,y1+1,x2,y2+1,0xff00ff00);
    aalineColor(renderer,x1,y1-1,x2,y2-1,0xff00ff00);

    // draw altitude
    if (orig_alt==0) orig_alt=ins_alt;
    double A=ins_alt-orig_alt;
    double TA=ins_target_alt-orig_alt;
    vlineColor(renderer,630,0,480,0xff00ff00);
    int i;
    hlineColor(renderer,590,630,430,0xff00ff00);
    for (i=0; i<=8; i++) {
        hlineColor(renderer,630,640,430-i*100,0xff00ff00);
    }
    hlineColor(renderer,590,630,430-A*10.0,0xff00ff00);
    hlineColor(renderer,590,630,430-A*10.0-1,0xff00ff00);
    hlineColor(renderer,590,630,430-A*10.0+1,0xff00ff00);

    char msg[256];
    sprintf(msg,"P=%.2f",P);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect2={10,10,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect2);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    sprintf(msg,"R=%.2f",R);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect3={10,40,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect3);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    sprintf(msg,"H=%.1f",H);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect4={10,70,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect4);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    sprintf(msg,"A=%.2f",A);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect5={10,100,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect5);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    sprintf(msg,"B=%.2f",ins_bat_volt);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect6={10,130,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect6);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    sprintf(msg,"T=%d",ins_throttle);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect9={10,160,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect9);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    sprintf(msg,"TP=%.2f",TP);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect7={150,10,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect7);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    sprintf(msg,"TR=%.2f",TR);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect8={150,40,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect8);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    sprintf(msg,"TA=%.2f",TA);
    font_surface=TTF_RenderText_Blended(font,msg,fg_color);
    font_texture=SDL_CreateTextureFromSurface(renderer,font_surface);
    SDL_QueryTexture(font_texture,NULL,NULL,&font_w,&font_h);
    SDL_Rect font_rect10={150,70,font_w,font_h};
    SDL_RenderCopy(renderer,font_texture,NULL,&font_rect10);
    SDL_DestroyTexture(font_texture);
    SDL_FreeSurface(font_surface);

    SDL_RenderPresent(renderer);

    SDL_DestroyRenderer(renderer);
}

int decode_video_packet(lua_State *L) {
    AVPacket pkt;
    int got_picture;
    int len;
    //printf("decode_video_packet\n");
    av_init_packet(&pkt);
    pkt.data=(uint8_t*)lua_tolstring(L,1,(size_t*)&pkt.size);
    while (pkt.size>0) {
        len=avcodec_decode_video2(codec_ctx,picture,&got_picture,&pkt);
        //printf("len=%d\n",len);
        if (len<0) {
            printf("Error decoding video frame!\n");
            break;
        }
        if (got_picture) {
            //printf("rendering frame %dx%d\n",picture->width,picture->height);
            got_frame=1;
        } else {
            got_frame=0;
        }
        pkt.size-=len;
        pkt.data+=len;
    }
    return 0;
}

int draw_text(lua_State *L) {
    size_t size;
    uint8_t *buf;
    buf=(uint8_t*)lua_tolstring(L,1,&size);
    strncpy(screen_text,buf,size);
    screen_text[size]=0;
    printf("draw_text '%s'\n",screen_text);
    //render_screen();
    return 0;
}

int update_instruments(lua_State *L) {
    size_t size;
    uint8_t *buf;
    ins_imu_rxz=(double)lua_tointeger(L,1)/16384.0;
    ins_imu_ryz=(double)lua_tointeger(L,2)/16384.0;
    ins_comp_x=(double)lua_tointeger(L,3)/16384.0;
    ins_comp_y=(double)lua_tointeger(L,4)/16384.0;
    ins_alt=(double)lua_tointeger(L,5)/100.0;
    ins_bat_volt=(double)lua_tointeger(L,6)/100.0;
    ins_target_rxz=(double)lua_tointeger(L,7)/16384.0;
    ins_target_ryz=(double)lua_tointeger(L,8)/16384.0;
    ins_throttle=lua_tointeger(L,9);
    ins_target_alt=(double)lua_tointeger(L,10)/100.0;
    return 0;
}

int lua_render_screen(lua_State *L) {
    render_screen();
    return 0;
}

void init_video_decode() {
    int res;
    avcodec_register_all();
    codec=avcodec_find_decoder(CODEC_ID_H264);
    if (!codec) {
        printf("video codec not found!\n");
        exit(1);
    }
    codec_ctx=avcodec_alloc_context3(codec);
    codec_ctx->flags|=CODEC_FLAG_LOW_DELAY;
    codec_ctx->flags2|=CODEC_FLAG2_CHUNKS;
    res=avcodec_open2(codec_ctx,codec,NULL);
    picture=avcodec_alloc_frame();
    if (res<0) {
        printf("failed to open codec!\n");
        exit(1);
    }
}

void set_drone_address(char *host, int port) {
    int res;
    lua_getglobal(L,"set_drone_address");
    lua_pushlstring(L,host,strlen(host));
    lua_pushnumber(L,port);
    res=lua_pcall(L,2,0,0);
    if (res!=0) {
        error(L,"lua error: %s",lua_tostring(L,-1));
    }
}

int main(int argc, char **argv) {
    int res;
    float display_w;
    float border;
    SDL_Joystick *joy;
    SDL_Event event;

    res=SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK);
    if (res<0) {
        printf("failed to init SDL\n");
        exit(1);
    }
    res=SDL_NumJoysticks();
    printf("num_joysticks=%d\n",res);
    if (res<=0) {
        printf("joystick not found\n");
        exit(1);
    }

    SDL_JoystickEventState(SDL_ENABLE);
    joy=SDL_JoystickOpen(0);

    window=SDL_CreateWindow("Raspilot",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED,640,480,SDL_WINDOW_SHOWN|SDL_WINDOW_RESIZABLE);
   // window=SDL_CreateWindow("Raspilot",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED,0,0,SDL_WINDOW_FULLSCREEN_DESKTOP);
    if (!window) {
        printf("failed to create window\n");
        exit(1);
    }

    TTF_Init();
    font = TTF_OpenFont("DroidSans.ttf", 25);

    init_video_decode();

    L=luaL_newstate();
    luaL_openlibs(L);
    lua_register(L,"draw_text",draw_text);
    lua_register(L,"update_instruments",update_instruments);
    lua_register(L,"decode_video_packet",decode_video_packet);
    lua_register(L,"render_screen",lua_render_screen);

    res=luaL_dofile(L,"raspilot.lua");
    if (res!=0) {
        error(L,"lua error: %s",lua_tostring(L,-1));
        return -1;
    }

    if (argc<3) {
        printf("Usage: raspilot DRONE_HOST DRONE_PORT\n");
        return -1;
    }
    char *drone_host=argv[1];
    int drone_port=atoi(argv[2]);
    set_drone_address(drone_host,drone_port);

    while (1) {
        int t;
        const char *key_name;
        res=SDL_WaitEventTimeout(&event,10);
        t=SDL_GetTicks();
        SDL_StartTextInput();
        if (res) {
            switch (event.type) {
                case SDL_KEYDOWN:
                    key_name=SDL_GetKeyName(event.key.keysym.sym);
                    printf("key down %d %s\n",event.key.keysym.scancode,key_name);
                    int code=event.key.keysym.scancode;
                    if (code==79) {
                        on_key('l',0,0);
                    } else if (code==80) {
                        on_key('h',0,0);
                    } else if (code==81) {
                        on_key('j',0,0);
                    } else if (code==82) {
                        on_key('k',0,0);
                    }
                    break;
                case SDL_TEXTINPUT:
                    printf("#################################################################\n");
                    printf("text: %s\n",event.text.text);
                    on_key(event.text.text[0],0,0);
                    break;
                case SDL_JOYAXISMOTION:
                    //printf("joy axis %d %d\n",event.jaxis.axis,event.jaxis.value);
                    if (event.jaxis.axis==0) {
                        joy_x1=event.jaxis.value;
                    } else if (event.jaxis.axis==1) {
                        joy_y1=-event.jaxis.value;
                    } else if (event.jaxis.axis==3) {
                        joy_x2=event.jaxis.value;
                    } else if (event.jaxis.axis==4) {
                        joy_y2=-event.jaxis.value;
                    }
                    joy_axis_changed=1;
                    break;
                case SDL_JOYBUTTONDOWN:
                    //printf("joy button %d\n",event.jbutton.button);
                    on_joy_button(event.jbutton.button);
                    break;
                case SDL_JOYHATMOTION:
                    //printf("joy hat %d\n",event.jhat.value);
                    on_joy_hat(event.jhat.value);
                    break;
                case SDL_QUIT:
                    printf("quit");
                    exit(0);
                    break;
            }
        }
        if (!prev_joy_axis_change_t || t-prev_joy_axis_change_t>=JOY_MAX_INTERVAL) {
            on_joy_axis();
            prev_joy_axis_change_t=t;
            joy_axis_changed=0;
        } else if (joy_axis_changed) {
            if (!prev_joy_axis_change_t || t-prev_joy_axis_change_t>=JOY_MIN_INTERVAL) {
                on_joy_axis();
                prev_joy_axis_change_t=t;
                joy_axis_changed=0;
            }
        }
        if (!prev_timer_t || t-prev_timer_t>=10) {
            on_timer();
            prev_timer_t=t;
        }
    }

    return 0;
}
