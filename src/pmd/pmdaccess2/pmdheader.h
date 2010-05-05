#pragma once

enum FilterTypes {
        NO_FILTER = 0,
        MEDIAN_FILTER,       //1
        MEAN_FILTER,         //2
        ADAPTIVE_MEAN_FILTER //3
};



typedef struct ImageHeaderInformation {
   float    DataSize;
   float    HeaderSize;
   float    ImageType;
   float    SamplingMode;
   float    SamplingRate;
   float    IntegrationTime_Exp0;
   float    IntegrationTime_Exp1;
   float    Seconds;
   float    Useconds;
   float    MedianFilter;
   float    MeanFilter;
   float    AdaptiveMeanFilter;
   float    ExponentialFilter;
   float    ExponentialFilterValue;
   float    AdaptiveMeanValueFilterThreshold;
   float    ValidImage;
   float    ErrorCode;
   float    Temperature;
   float    CurrentProgram;
   float    CurrentValue;
   float    CurrentTriggerMode;
   float    CurrentProduct;
   float    CurrentUnit;
   float    RoiLeft;
   float    RoiRight;
   float    RoiTop;
   float    RoiBottom;
   float    FocalDistance;
   float    PixelSize;
   float    DelayStartTime;
   float    DelayReverseTime;
   float    OutputTypeOut1;
   float    OutputTypeOut2;
   float    internaluse01;
   float    internaluse02;
   float    internaluse03;
   float    internaluse04;
   float    internaluse05;
   float    internaluse06;
   float    internaluse07;
   float    internaluse08;
   float    internaluse09;
   float    internaluse10;
   float    internaluse11;
   float    internaluse12;
   float    internaluse13;
   float    internaluse14;
   float    internaluse15;
   float    internaluse16;
   float    internaluse17;
   float    internaluse18;
   float    internaluse19;
   float    internaluse20;
   float    internaluse21;
   float    internaluse22;
   float    internaluse23;
   float    internaluse24;
   float    AverageDetermination;
   float    Reserved1;
   float    Reserved2;
   float    Reserved3;
   float    Reserved4;
   float    Reserved5;
   float    Reserved6;
   float    Reserved7;
   float    Reserved8;
   float    Reserved9;
   float    Reserved10;
   float    Reserved11;
   float    Reserved12;
   float    Reserved13;
   float    Reserved14;
   float    Reserved15;
   float    Reserved16;
   float    Reserved17;
   float    Reserved18;
   float    Reserved19;
   float    Reserved20;
   float    Reserved21;
   float    Reserved22;
   float    Reserved23;
   float    Reserved24;
   float    Reserved25;
   float    Reserved26;
   float    Reserved27;
   float    Reserved28;
   float    Reserved29;
   float    Reserved30;
   float    Reserved31;
   float    Reserved32;
   float    Reserved33;
   float    Reserved34;
   float    Reserved35;
   float    Reserved36;
   float Reserved37;
   float Reserved38;
   float Reserved39;
   float Reserved40;
   float Reserved41;
   float Reserved42;
   float Reserved43;
   float Reserved44;
   float Reserved45;
} T_IMAGEHEADER;

enum ImageTypes {
      INVALID_IMAGE = 0,
      DISTANCE_IMAGE ,       //1
      INTENSITY_IMAGE,       //2
      NOT_USED_TYPE,         //3
      CURRENT_STD_IMAGE,     //4
      NORMAL_X_IMAGE,        //5
      NORMAL_Y_IMAGE,        //6
      NORMAL_Z_IMAGE,        //7
      KARTESIAN_X_IMAGE,     //8
      KARTESIAN_Y_IMAGE,     //9
      KARTESIAN_Z_IMAGE     //10
};

