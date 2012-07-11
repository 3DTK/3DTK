/*
 * pmdaccess implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#include "pmddatadescription.h"
#include "pmdsdk2.h"
#include "pmdheader.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <strings.h>
#include <string.h>
#include <stdio.h>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/client_simple.hpp>



int sockfd;

ImageHeaderInformation header;

// retrive latest header
ImageHeaderInformation *retriveHeader() {
    return &header;
}


int pmdOpen(PMDHandle *hnd, const char *rplugin, const char *rparam, const char *pplugin, const char *pparam) {

    printf("Starting XMLRPC...\n"); //DEBUG

    char url[128];
    strcpy(url, rparam);
    strcat(url, ":8080");
    xmlrpc_c::clientSimple myClient;    
    xmlrpc_c::value result;
    xmlrpc_c::paramList ipAndHeart;
    ipAndHeart.add(xmlrpc_c::value_string("192.168.0.1"));
    ipAndHeart.add(xmlrpc_c::value_int(1));
    myClient.call( url
                 , "MDAXMLConnectCP"
                 , ipAndHeart
                 , &result);

    // start data server
    xmlrpc_c::paramList dataServ;
    xmlrpc_c::value result2;
    dataServ.add(xmlrpc_c::value_int(1));
    myClient.call( url
                 , "MDAXMLSetWorkingMode"
                 , dataServ
                 , &result2);

    printf("Done XMPRC init, creating socket...\n"); //DEBUG


    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    server = gethostbyname(rparam);
    serv_addr.sin_family = AF_INET;

    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(50002);
    if (connect( sockfd
               , (struct sockaddr *)&serv_addr
               , sizeof(serv_addr)) 
            == -1) {
        perror("Error connecting!\n");
        return 1;
    }
    printf("Connceted!\n"); //DEBUG

    return 0;
}


int pmdClose (PMDHandle hnd) {

    xmlrpc_c::clientSimple myClient;    
    xmlrpc_c::value result;
    xmlrpc_c::paramList ip;
    ip.add(xmlrpc_c::value_string("192.168.0.1"));
    myClient.call( "192.168.0.69:8080" //FIXME: hardcoded ip
                 , "MDAXMLDisconnectCP"                  
                 , ip, &result);

    return 0;
}

int pmdGetLastperror (PMDHandle hnd, char *perror, size_t maxLen) {
    return 0;
}

int pmdUpdate(PMDHandle hnd) {
    xmlrpc_c::clientSimple myClient;    
    xmlrpc_c::value result;
    xmlrpc_c::paramList noArgs;
    //FIXME: There is no HearBeat XML function!!!
    //myClient.call( "192.168.0.69:8080", "MDAXMLHeartBeat" //FIXME: hardcoded ip
    //             , noArgs, &result);

    return 0;
}

// Currently will work for one image only (i.e. "d" or "i" but not for "id")
int pmdGetData(const char *code, PMDHandle hnd, float *data, size_t maxLen) {
    int n = write(sockfd, code, strlen(code));
    if (n < 0) 
         perror("Error writing to socket\n");

    // reading image header
    uint32_t headerBuf[412/4]; // 412 bytes `div` sizeof(uint32_t)
    int bytesRead = 0;
    while(bytesRead < 412) {
        n = read(sockfd, &((char*)headerBuf)[bytesRead], 412 - bytesRead);
        if (n < 0)
             perror("Error reading from socket\n");
        bytesRead += n;

    }    
    for(int i = 0; i < 412/4; i++)
        ((uint32_t*)&header)[i] = ntohl(headerBuf[i]);

    // reading image
    uint32_t imgBuf[maxLen/4]; // maxLen is also in bytes
    bytesRead = 0;
    while(bytesRead < maxLen) {
        n = read(sockfd, &((char*)imgBuf)[bytesRead], maxLen - bytesRead);
        if (n < 0) 
          perror("Error reading from socket\n");
        bytesRead += n;
    }

    for(int i = 0; i < maxLen/4; i++) 
        ((uint32_t*)data)[i] = ntohl(imgBuf[i]);

    return 0;
}

int pmdGetDistances(PMDHandle hnd, float *data, size_t maxLen) {
    return pmdGetData("D", hnd, data, maxLen);
}


int pmdGetAmplitudes (PMDHandle hnd, float * data, size_t maxLen) {
    return pmdGetData("I", hnd, data, maxLen);
}

int pmdGetIntensities (PMDHandle hnd, float * data, size_t maxLen) {
    return pmdGetData("S", hnd, data, maxLen);
}

int pmdGetDistancesAsync(PMDHandle hnd, float *data, size_t maxLen) {
    return pmdGetData("d", hnd, data, maxLen);
}

int pmdGetAmplitudesAsync(PMDHandle hnd, float * data, size_t maxLen) {
    return pmdGetData("i", hnd, data, maxLen);
}

int pmdGetIntensitiesAsync(PMDHandle hnd, float * data, size_t maxLen) {
    return pmdGetData("s", hnd, data, maxLen);
}

