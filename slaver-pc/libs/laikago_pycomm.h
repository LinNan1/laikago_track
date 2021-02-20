#ifndef __LAIKAGO_PYCOMM_H__
#define __LAIKAGO_PYCOMM_H__
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

#include "laikago_comm.h"

#define BUFFER_SIZE 5000
#define LISTEN_PORT 8001

HighCmd recvbuf[BUFFER_SIZE];
sem_t mutex;
int front = 0, rear = 0;

int pushCmd(HighCmd cmd){
    int index = (rear+1) % BUFFER_SIZE;
    if(index == front){ // 满了,丢掉队首
        front = (front + 1) % BUFFER_SIZE;
    }
    recvbuf[rear]  = cmd;
    rear = index;
    return 0;
}
int frontCmd(HighCmd *cmd){
    if(front == rear) return 1;
    *cmd = recvbuf[front];
    front = (front + 1) % BUFFER_SIZE;
    return 0;
}

void *udp_recv(void *arg){
    struct sockaddr_in sin;
    memset(&sin,0,sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons(LISTEN_PORT);
    int msock = socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
    bind(msock,(sockaddr *)&sin,sizeof(sin));

    sockaddr_in from;
    unsigned int fromsize = sizeof(from);
    HighCmd tmp_recv;
    while(true){
        int len = recvfrom(msock,&tmp_recv,sizeof(HighCmd),0,(sockaddr*)&from,&fromsize);
        if(len>0){
            // printf("type: %d\n",recvbuf.type);
            // printf("val : %f\n",recvbuf.val);
            sem_wait(&mutex);
            pushCmd(tmp_recv);
            sem_post(&mutex);
        }
    }
}

void udp_send(HighStatus &highstat, const char* addr, unsigned int port){
    struct sockaddr_in sin;
    memset(&sin,0,sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = inet_addr(addr);
    sin.sin_port = htons(port);
    int msock = socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
    sendto(msock, &highstat, sizeof(HighStatus), 0, (sockaddr*)&sin, sizeof(sin));
}

pthread_t pycommInit(void){
    sem_init(&mutex,0,1);
    pthread_t tid;
    pthread_create(&tid, NULL, udp_recv, NULL);
    return tid;
}

int getCmd(HighCmd *cmd){
    sem_wait(&mutex);
    int flag = frontCmd(cmd);
    sem_post(&mutex);
    return flag;
}

int sendStat(HighStatus &highstat, const char* addr, unsigned int port){
    udp_send(highstat, addr, port);
    return 0;
}

#endif
